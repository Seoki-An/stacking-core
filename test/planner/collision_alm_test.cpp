#include "collision_alm.hpp"

#include <stacking_core/collision.hpp>
#include <stacking_core/planner/motion.hpp>

#include <cmath>
#include <iostream>
#include <memory>
#include <source_location>
#include <stdexcept>
#include <string>

namespace {

  using namespace stacking_core;
  using namespace stacking_core::planner_detail;

  void require(bool ok, std::source_location loc = std::source_location::current()) {
    if (!ok) {
      throw std::runtime_error("collision ALM test line " + std::to_string(loc.line()));
    }
  }

  void legacy_phr_equations() {
    // Same PHR expressions as diffsim free/grasped process_pair. Include
    // active, inactive, and nonzero-multiplier cases; no dt scaling.
    for (Scalar lambda : {0.0, 0.3, 2.0}) {
      for (Scalar beta : {2.0, 100.0, 500.0}) {
        for (Scalar c : {-0.03, -0.0041, 0.002, 0.01}) {
          Scalar const active = std::max(0.0, lambda + beta * c);
          auto const actual = collision_alm_term(lambda, beta, c);
          Scalar const expected = (active * active - lambda * lambda) / (2.0 * beta);
          require(std::abs(actual.cost - expected) < 1e-14);
          require(std::abs(actual.d_violation - active) < 1e-14);
          Scalar const eps = 1e-7;
          Scalar const fd = (collision_alm_term(lambda, beta, c + eps).cost -
                            collision_alm_term(lambda, beta, c - eps).cost) / (2 * eps);
          require(std::abs(fd - actual.d_violation) < 1e-8);
        }
      }
    }
    collision_key_t const first {3, EntityId {1}, EntityId {2}};
    collision_key_t const other {4, EntityId {1}, EntityId {2}};
    collision_alm_state_t state {.beta = 100.0, .duals = {}};
    state.update({{first, 0.01}, {other, 0.02}});
    require(state.multiplier(first) == 1.0 && state.multiplier(other) == 2.0);
    state.update({{first, -0.004}});
    require(std::abs(state.multiplier(first) - 0.6) < 1e-14);
    require(state.multiplier(other) == 0.0);
    state.update({{first, -0.01}});
    require(state.multiplier(first) == 0.0);
  }

  std::shared_ptr<BodyModel const> ball(Scalar radius, bool nested = false) {
    Matrix3X nodes(3, 6);
    nodes << radius, -radius, 0, 0, 0, 0,
      0, 0, radius, -radius, 0, 0,
      0, 0, 0, 0, radius, -radius;
    dsf_vert_geometry_config_t geom;
    geom.properties.id = GeometryId {1};
    geom.nodes = nodes;
    geom.sharpness = 2;
    body_model_config_t model;
    model.id = BodyModelId {1};
    model.geometries.push_back(geom);
    if (nested) {
      geom.properties.id = GeometryId {2};
      geom.nodes *= 0.5;
      model.geometries.push_back(geom);
    }
    return std::make_shared<BodyModel>(std::move(model));
  }

  motion_robot_t cartesian_robot(std::shared_ptr<BodyModel const> const& model) {
    kinematic_model_config_t kin;
    kin.links = {{LinkId {1}, "base"}, {LinkId {2}, "x"}, {LinkId {3}, "tool"}};
    for (int i = 0; i < 2; ++i) {
      kinematic_joint_t joint;
      joint.id = JointId {static_cast<std::uint64_t>(i + 1)};
      joint.name = std::to_string(i);
      joint.type = joint_type_e::prismatic;
      joint.parent = kin.links[i].id;
      joint.child = kin.links[i + 1].id;
      joint.axis = i == 0 ? Vector3::UnitX() : Vector3::UnitZ();
      joint.limit = joint_limit_t {-1.0, 1.0};
      kin.joints.push_back(joint);
    }
    KinematicState state {FrameId {1}, std::make_shared<KinematicModel>(std::move(kin))};
    state.setPositions((Eigen::Vector2d() << -0.3, 0.06).finished());
    return motion_robot_t {
      .initial_state = state,
      .tool_link = LinkId {3},
      .link_from_tool = {},
      .gripper_opening = 0.0,
      .ik_initializer = {},
      .collision_bodies = {{LinkId {3}, EntityId {2}, model}},
      .self_collision_pairs = {},
    };
  }

  void obstacle_detour() {
    auto const carried = ball(0.03);
    auto robot = cartesian_robot(carried);
    std::vector<BodyInstance> bodies;
    bodies.emplace_back(body_instance_config_t {
      .id = EntityId {1}, .model = ball(0.15), .frame_from_body = {},
      .motion = {}, .mobility = mobility_e::static_body});
    bodies.emplace_back(body_instance_config_t {
      .id = EntityId {3}, .model = carried,
      .frame_from_body = pose_t {Vector3 {-0.3, 0, 0.06}, Quaternion::Identity()},
      .motion = {}, .mobility = mobility_e::static_body});
    auto snapshot = std::make_shared<SceneSnapshot>(scene_snapshot_config_t {
      .frame = FrameId {1}, .bodies = std::move(bodies)});
    SceneView free_scene {snapshot, {EntityId {1}}};
    SceneView held_scene {snapshot, {EntityId {1}, EntityId {3}}};
    motion_waypoint_t goal {
      .goal = joint_goal_t {.positions = Eigen::Vector2d {0.3, 0.06}}, .steps = 16};
    motion_planning_config_t config;
    config.smooth_weight = 300;
    config.collision_margin = 0.01;
    config.collision_penetration_clamp = 0.01;
    config.max_iters = 400;
    config.tol = 1e-7;
    auto plain = solve_free_motion({free_scene, robot, {goal}}, config);
    config.collision_alm_enabled = true;
    config.collision_alm_max_iters = 10;
    config.collision_alm_beta_init = 100;
    config.collision_alm_beta_increase = 5;
    config.collision_alm_tol = 0.0; // Keep this test's exact final margins.
    auto bounded_config = config;
    bounded_config.collision_alm_max_iters = 1;
    auto bounded = solve_free_motion({free_scene, robot, {goal}}, bounded_config);
    require(bounded.status == solve_status_e::infeasible);
    require(bounded.failure.code == "collision_or_joint_limit");
    auto enforced = solve_free_motion({free_scene, robot, {goal}}, config);
    // The held target follows the same Cartesian path. No robot collision
    // geometry is needed for this test of target-vs-scene constraints.
    robot.collision_bodies.clear();
    config.target_collision_tol = 0.0;
    auto held = solve_grasped_motion({held_scene, robot,
      attachment_t {EntityId {3}, LinkId {3}, pose_t {}}, {goal}}, config);
    std::cout << "plain=" << int(plain.status) << " " << plain.failure.message
      << "\nfree_alm=" << int(enforced.status) << " " << enforced.failure.message
      << "\nheld_alm=" << int(held.status) << " " << held.failure.message << '\n';
    require(plain.status == solve_status_e::infeasible);
    require(enforced.status == solve_status_e::success);
    require(held.status == solve_status_e::success);
    for (auto const* result : {&enforced, &held}) {
      require(result->trajectory.samples.front().robot.positions.isApprox(
        robot.initial_state.positions(), 1e-12));
      for (auto const& sample : result->trajectory.samples) {
        // Independent analytic ball-ball gap for this DSF(p=2) fixture.
        Scalar const gap = sample.robot.positions.norm() - 0.18;
        require(gap >= -1e-9);
      }
    }
    auto repeated = solve_free_motion({free_scene, cartesian_robot(carried), {goal}}, config);
    require(repeated.status == enforced.status);
    require(repeated.solver.iters == enforced.solver.iters);
    for (std::size_t i = 0; i < enforced.trajectory.samples.size(); ++i) {
      require(repeated.trajectory.samples[i].robot.positions.isApprox(
        enforced.trajectory.samples[i].robot.positions, 1e-12));
    }

    // With clearance disabled, adding a nested convex piece must not add a
    // second PHR penalty: legacy constrains only the pair's deepest feature.
    bounded_config.collision_weight = 0.0;
    auto single = solve_free_motion(
      {free_scene, cartesian_robot(carried), {goal}}, bounded_config);
    auto compound = solve_free_motion(
      {free_scene, cartesian_robot(ball(0.03, true)), {goal}}, bounded_config);
    require(single.status == compound.status);
    require(single.solver.iters == compound.solver.iters);
    for (std::size_t i = 0; i < single.trajectory.samples.size(); ++i) {
      require(single.trajectory.samples[i].robot.positions.isApprox(
        compound.trajectory.samples[i].robot.positions, 1e-12));
    }
  }

} // namespace

int main() {
  legacy_phr_equations();
  obstacle_detour();
}
