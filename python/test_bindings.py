import unittest

import numpy as np

import stacking_core


def identity_pose(z: float = 0.0) -> np.ndarray:
    return np.array([0.0, 0.0, z, 0.0, 0.0, 0.0, 1.0])


class SimulationBindingsTest(unittest.TestCase):
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


if __name__ == "__main__":
    unittest.main()
