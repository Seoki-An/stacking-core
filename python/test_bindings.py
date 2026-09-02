import gc
from pathlib import Path
import threading
import unittest

import numpy as np

import stacking_core


ASSET_DIR = Path(__file__).resolve().parent / "assets"
if not ASSET_DIR.exists():
    ASSET_DIR = Path(__file__).resolve().parents[1] / "test" / "planner" / "assets"


def identity_pose(z: float = 0.0) -> np.ndarray:
    return np.array([0.0, 0.0, z, 0.0, 0.0, 0.0, 1.0])


class SimulationBindingsTest(unittest.TestCase):
    def test_posegen_wrench_matrix_round_trips(self) -> None:
        objective = stacking_core.PosegenObjectiveConfig()
        expected = 2.0 * np.eye(6)

        objective.k_wrench = expected

        self.assertTrue(np.array_equal(objective.k_wrench, expected))
        with self.assertRaisesRegex(ValueError, "k_wrench must have shape"):
            objective.k_wrench = np.eye(3)

    def test_pose_generator_returns_immutable_native_result(self) -> None:
        nodes = np.array(
            [
                [-0.5, -0.5, -0.5],
                [-0.5, -0.5, 0.5],
                [-0.5, 0.5, -0.5],
                [-0.5, 0.5, 0.5],
                [0.5, -0.5, -0.5],
                [0.5, -0.5, 0.5],
                [0.5, 0.5, -0.5],
                [0.5, 0.5, 0.5],
            ]
        )
        cube = stacking_core.BodyModel.dsf(
            1, nodes, mass=1.0, inertia=np.eye(3)
        )
        scene = stacking_core.SceneSnapshot(
            1,
            [stacking_core.BodyInstance(1, cube, identity_pose(1.5))],
        )

        result = stacking_core.PoseGenerator().solve(
            stacking_core.PosegenProblem(scene, candidate=1)
        )

        self.assertEqual(result.optimal_pose.shape, (7,))
        self.assertTrue(np.all(np.isfinite(result.optimal_pose)))
        self.assertTrue(result.solver.converged)
        self.assertIn(1, result.net_wrench)

    def test_pose_uses_scalar_last_quaternion_at_python_boundary(self) -> None:
        plane = stacking_core.BodyModel.plane(1)
        quaternion_xyzw = np.array([0.2, -0.3, 0.4, 0.5])
        quaternion_xyzw /= np.linalg.norm(quaternion_xyzw)
        pose = np.concatenate(([1.0, 2.0, 3.0], quaternion_xyzw))
        scene = stacking_core.SceneSnapshot(
            1,
            [
                stacking_core.BodyInstance(
                    1,
                    plane,
                    pose,
                    mobility=stacking_core.Mobility.STATIC,
                )
            ],
        )

        self.assertTrue(np.allclose(scene.body_pose(1), pose, atol=1e-12))

    def test_scene_step_returns_canonical_snapshot(self) -> None:
        plane = stacking_core.BodyModel.plane(1, friction=0.6)
        cube = stacking_core.BodyModel.dsf(
            2,
            np.array(
                [
                    [-0.5, -0.5, -0.5],
                    [-0.5, -0.5, 0.5],
                    [-0.5, 0.5, -0.5],
                    [-0.5, 0.5, 0.5],
                    [0.5, -0.5, -0.5],
                    [0.5, -0.5, 0.5],
                    [0.5, 0.5, -0.5],
                    [0.5, 0.5, 0.5],
                ]
            ),
            mass=1.0,
            inertia=np.eye(3),
            friction=0.6,
            sharpness=100,
        )
        scene = stacking_core.SceneSnapshot(
            1,
            [
                stacking_core.BodyInstance(
                    1,
                    plane,
                    identity_pose(),
                    mobility=stacking_core.Mobility.STATIC,
                ),
                stacking_core.BodyInstance(2, cube, identity_pose(1.0)),
            ],
        )

        result = stacking_core.Simulator().step(scene, 0.01)

        self.assertEqual(result.snapshot.frame_id, 1)
        self.assertEqual(result.snapshot.body_count, 2)
        self.assertLess(result.snapshot.body_pose(2)[2], 1.0)
        self.assertEqual(result.snapshot.body_pose(2).shape, (7,))
        self.assertTrue(result.solver.converged)

    def test_invalid_pose_shape_is_rejected(self) -> None:
        plane = stacking_core.BodyModel.plane(1)
        with self.assertRaisesRegex(ValueError, "pose must contain"):
            stacking_core.BodyInstance(1, plane, np.zeros(6))

    def test_body_model_accepts_multiple_dsf_components(self) -> None:
        first = np.array(
            [
                [-1.0, -1.0, -1.0],
                [1.0, -1.0, -1.0],
                [-1.0, 1.0, -1.0],
                [-1.0, -1.0, 1.0],
            ]
        )
        second = first + np.array([3.0, 0.0, 0.0])

        model = stacking_core.BodyModel.dsf_components(
            2,
            [first, second],
            mass=1.0,
            inertia=np.eye(3),
            frictions=[0.5, 0.7],
            sharpnesses=[8, 12],
        )

        self.assertEqual(model.geometry_count, 2)
        self.assertEqual(model.mass, 1.0)


class PlannerBindingsTest(unittest.TestCase):
    def make_pick_place_problem(self):
        target_urdf = stacking_core.load_urdf_model(
            str(ASSET_DIR / "stable_pose.urdf")
        )
        target_link = target_urdf.kinematics.find_link("target")
        target_model = target_urdf.body_model(target_link)

        def phase():
            snapshot = stacking_core.SceneSnapshot(
                1,
                [
                    stacking_core.BodyInstance(
                        900,
                        target_model,
                        identity_pose(),
                        mobility=stacking_core.Mobility.KINEMATIC,
                    )
                ],
            )
            return stacking_core.PhaseScene(
                stacking_core.SceneView(snapshot, [900]), 900
            )

        robot_urdf = stacking_core.load_urdf_model(
            str(ASSET_DIR / "excavator_kinematics.urdf")
        )
        model = robot_urdf.kinematics
        tool_link = model.find_link("cs_rotate")
        state = stacking_core.KinematicState(1, model)
        positions = np.array([0.1, 0.2, -0.2, -0.1, 0.1, 0.0])
        state.positions = positions

        robot = stacking_core.MotionRobot(
            state,
            tool_link,
            identity_pose(),
            gripper_opening=-0.2,
        )
        gripper = stacking_core.GripperModel(
            state,
            model.roots[0],
            identity_pose(),
            np.zeros(model.dof_count),
            np.zeros(model.dof_count),
            opening_lower=-0.2,
            opening_upper=0.0,
        )
        grasp = stacking_core.Grasp(state.link_pose(tool_link), opening=-0.1)
        grasp_candidate = stacking_core.GraspCandidate()
        grasp_candidate.grasp = grasp
        grasp_candidate.score = 3.0
        joint_candidate = stacking_core.JointGraspCandidate()
        joint_candidate.grasp = grasp_candidate
        joint_candidate.positions = [positions, positions]
        direct = stacking_core.DirectPlanProblem(
            phase(), phase(), robot, gripper, [joint_candidate]
        )
        return stacking_core.PickPlaceProblem(
            direct,
            phase(),
            np.array([2.0, 3.0, 0.25]),
        )

    @staticmethod
    def direct_only_config():
        config = stacking_core.PickPlaceConfig()
        config.allow_regrasp = False
        config.direct.simulation_refinement = False
        config.direct.approach_distance = 0.0
        config.direct.move_steps = 2
        config.direct.grasp_steps = 2
        config.direct.motion.max_iters = 1
        return config

    def test_scene_view_retains_snapshot(self) -> None:
        model = stacking_core.BodyModel.plane(21)
        snapshot = stacking_core.SceneSnapshot(
            8,
            [stacking_core.BodyInstance(12, model, identity_pose())],
        )
        phase = stacking_core.PhaseScene(
            stacking_core.SceneView(snapshot, [12]), 12
        )

        del snapshot
        gc.collect()

        self.assertEqual(phase.scene.frame_id, 8)
        self.assertEqual(phase.scene.entity_ids, [12])
        self.assertEqual(phase.scene.snapshot.body_count, 1)

    def test_urdf_state_robot_and_gripper_construction(self) -> None:
        urdf = stacking_core.load_urdf_model(
            str(ASSET_DIR / "excavator_kinematics.urdf")
        )
        model = urdf.kinematics
        tool_link = model.find_link("cs_rotate")
        state = stacking_core.KinematicState(3, model)
        state.positions = np.zeros(model.dof_count)
        tool_body = urdf.body_model(tool_link)
        motion_body = stacking_core.MotionLinkBody(tool_link, 101, tool_body)
        grasp_body = stacking_core.GraspLinkBody(tool_link, 201, tool_body)
        contact_geometry = stacking_core.GraspContactGeometry(
            201,
            1,
            stacking_core.GraspContactSide.LEFT,
            stacking_core.GraspContactType.PAD,
        )

        robot = stacking_core.MotionRobot(
            state,
            tool_link,
            identity_pose(),
            collision_bodies=[motion_body],
            self_collision_pairs=[stacking_core.CollisionBodyPair(101, 102)],
        )
        gripper = stacking_core.GripperModel(
            state,
            model.roots[0],
            identity_pose(),
            np.zeros(model.dof_count),
            np.zeros(model.dof_count),
            collision_bodies=[grasp_body],
            contact_geometries=[contact_geometry],
        )

        self.assertEqual(robot.tool_link, tool_link)
        self.assertEqual(robot.collision_bodies[0].entity, 101)
        self.assertEqual(gripper.root_link, model.roots[0])
        self.assertEqual(gripper.contact_geometries[0].entity, 201)
        self.assertEqual(state.link_pose(tool_link).shape, (7,))

    def test_nested_planner_configuration_is_writable(self) -> None:
        config = stacking_core.PickPlaceConfig()

        config.direct.grasp_sampling.max_seeds = 9
        config.direct.grasp_generation.trust_region.max_iters = 17
        config.direct.grasp_generation.force.weight.moment = 8.0
        config.direct.inverse_kinematics.initialization = (
            stacking_core.InverseKinematicsInitialization.PROVIDED
        )
        config.direct.grasp_simulation.pgs_iters = 13
        config.direct.motion.collision_margin = 0.03
        config.regrasp.stable_pose.sampling_level = 4

        self.assertEqual(config.direct.grasp_sampling.max_seeds, 9)
        self.assertEqual(
            config.direct.grasp_generation.trust_region.max_iters, 17
        )
        self.assertEqual(config.direct.grasp_generation.force.weight.moment, 8.0)
        self.assertEqual(config.direct.grasp_simulation.pgs_iters, 13)
        self.assertEqual(config.regrasp.stable_pose.sampling_level, 4)

    def test_pick_place_success_result_matches_adapter_shape(self) -> None:
        result = stacking_core.solve_pick_place(
            self.make_pick_place_problem(), self.direct_only_config()
        )

        self.assertEqual(result.status, stacking_core.SolveStatus.SUCCESS)
        self.assertEqual(result.selected_index, 0)
        self.assertEqual(len(result.candidates), 1)
        candidate = result.candidates[result.selected_index]
        self.assertEqual(len(candidate.segments), 5)
        self.assertGreater(len(candidate.segments[0].trajectory.samples), 0)
        sample = candidate.segments[0].trajectory.samples[0]
        self.assertEqual(sample.robot.positions.shape, (6,))
        self.assertEqual(sample.frame_from_target.shape, (7,))

    def test_pick_place_failure_is_structured(self) -> None:
        config = self.direct_only_config()
        config.direct.move_steps = 1

        result = stacking_core.solve_pick_place(
            self.make_pick_place_problem(), config
        )

        self.assertEqual(result.status, stacking_core.SolveStatus.INVALID_PROBLEM)
        self.assertEqual(result.failure.code, "invalid_direct_config")
        self.assertTrue(result.failure.message)
        self.assertFalse(result.failure.retryable)
        self.assertIsNone(result.selected_index)

    def test_pick_place_releases_gil(self) -> None:
        problem = self.make_pick_place_problem()
        config = self.direct_only_config()
        config.direct.move_steps = 300
        started = threading.Event()
        stop = threading.Event()
        counter = [0]

        def worker() -> None:
            started.set()
            while not stop.is_set():
                counter[0] += 1

        thread = threading.Thread(target=worker)
        thread.start()
        started.wait()
        before = counter[0]
        result = stacking_core.solve_pick_place(problem, config)
        after = counter[0]
        stop.set()
        thread.join()

        self.assertEqual(result.status, stacking_core.SolveStatus.SUCCESS)
        self.assertGreater(after, before)


if __name__ == "__main__":
    unittest.main()
