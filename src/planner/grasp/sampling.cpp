#include <stacking_core/body/convex_support_envelope.hpp>
#include <stacking_core/collision.hpp>
#include <stacking_core/planner/grasp/sampling.hpp>

#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <iterator>
#include <limits>
#include <memory>
#include <numbers>
#include <optional>
#include <random>
#include <string>
#include <utility>
#include <vector>

namespace stacking_core {
  namespace {

    struct seed_parameter_t {
      Vector3 orientation_sample = Vector3::Zero();
      bool flip = false;
      Scalar spin = 0.0;
    };

    struct pad_profile_sample_t {
      Scalar opening = 0.0;
      Scalar aperture = 0.0;
      Vector3 midpoint = Vector3::Zero();
    };

    grasp_seed_result_t invalid_seed_result(
      std::string code, std::string message) {
      return grasp_seed_result_t {
        .status = solve_status_e::invalid_problem,
        .seeds = {},
        .rejected_width = 0,
        .rejected_clearance = 0,
        .failure =
          planner_failure_t {
            .code = std::move(code),
            .message = std::move(message),
            .retryable = false,
          },
      };
    }

    int zigzag(int index) {
      if (index == 0) {
        return 0;
      }
      int const magnitude = (index + 1) / 2;
      return index % 2 == 1 ? magnitude : -magnitude;
    }

    std::vector<Vector3> hemisphere_dirs(int count) {
      std::vector<Vector3> dirs;
      dirs.reserve(static_cast<std::size_t>(count));
      Scalar const golden = std::numbers::pi_v<Scalar> * (3.0 - std::sqrt(5.0));
      for (int i = 0; i < count; ++i) {
        Scalar const z =
          (static_cast<Scalar>(i) + 0.5) / static_cast<Scalar>(count);
        Scalar const radius = std::sqrt(std::max(0.0, 1.0 - z * z));
        Scalar const angle = golden * static_cast<Scalar>(i);
        dirs.emplace_back(
          radius * std::cos(angle), radius * std::sin(angle), z);
      }
      return dirs;
    }

    std::vector<seed_parameter_t> seed_parameters(
      grasp_sampling_config_t const& config) {
      std::vector<Vector3> orientation_samples;
      if (config.strategy == grasp_sampling_strategy_e::antipodal_surface) {
        orientation_samples = hemisphere_dirs(config.dir_samples);
      } else {
        constexpr Scalar tilt_step = std::numbers::pi_v<Scalar> / 12.0;
        for (int x_index = 0; x_index < 5; ++x_index) {
          for (int z_index = 0; z_index < 3; ++z_index) {
            orientation_samples.emplace_back(
              tilt_step * zigzag(x_index), tilt_step * zigzag(z_index), 0.0);
          }
        }
      }

      std::vector<seed_parameter_t> all;
      int const flip_count = config.include_flipped ? 2 : 1;
      all.reserve(
        orientation_samples.size() * static_cast<std::size_t>(flip_count) *
        static_cast<std::size_t>(config.spin_samples));
      for (Vector3 const& orientation_sample : orientation_samples) {
        for (int flip = 0; flip < flip_count; ++flip) {
          for (int spin = 0; spin < config.spin_samples; ++spin) {
            all.push_back(seed_parameter_t {
              .orientation_sample = orientation_sample,
              .flip = flip != 0,
              .spin = config.spin_step * static_cast<Scalar>(spin),
            });
          }
        }
      }
      if (static_cast<std::size_t>(config.max_seeds) >= all.size()) {
        return all;
      }

      std::random_device random_device;
      std::mt19937 generator(random_device());
      if (config.strategy == grasp_sampling_strategy_e::parallel_jaw) {
        std::shuffle(std::next(all.begin()), all.end(), generator);
      } else {
        std::shuffle(all.begin(), all.end(), generator);
      }
      all.resize(static_cast<std::size_t>(config.max_seeds));
      return all;
    }

    grasp_link_body_t const* link_body(
      gripper_model_t const& gripper, EntityId entity) {
      auto const iter = std::find_if(
        gripper.collision_bodies.begin(), gripper.collision_bodies.end(),
        [&](grasp_link_body_t const& body) { return body.entity == entity; });
      return iter == gripper.collision_bodies.end() ? nullptr : &*iter;
    }

    std::optional<pad_profile_sample_t> measure_pad_profile(
      gripper_model_t const& gripper, Scalar opening) {
      KinematicState state = gripper.state;
      state.setPositions(
        gripper.opening_offset + opening * gripper.opening_direction);
      state.setFrameFromRoot(gripper.root_link, gripper.grasp_from_root);
      KinematicSnapshot const snapshot = forward_kinematics(state);

      Vector3 positive_sum = Vector3::Zero();
      Vector3 negative_sum = Vector3::Zero();
      int positive_count = 0;
      int negative_count = 0;
      for (grasp_contact_geometry_t const& role : gripper.contact_geometries) {
        if (role.type != grasp_contact_type_e::pad) {
          continue;
        }
        grasp_link_body_t const* binding = link_body(gripper, role.entity);
        if (binding == nullptr) {
          continue;
        }
        Geometry const* geometry =
          binding->body_model->findGeometry(role.geometry);
        if (
          geometry == nullptr ||
          geometry->type() != geometry_type_e::dsf_vert) {
          continue;
        }
        pose_t const& frame_from_body = snapshot.frameFromLink(binding->link);
        pose_t const frame_from_geometry =
          compose(frame_from_body, geometry->bodyFromGeometry());
        Scalar const x = frame_from_geometry.position.x();
        if (std::abs(x) < 1e-9) {
          continue;
        }
        Scalar const side = x > 0.0 ? 1.0 : -1.0;
        auto const& dsf = static_cast<DsfVertGeometry const&>(*geometry);
        Vector3 const point =
          dsf.support(-side * Vector3::UnitX(), frame_from_body).s;
        if (side > 0.0) {
          positive_sum += point;
          ++positive_count;
        } else {
          negative_sum += point;
          ++negative_count;
        }
      }
      if (positive_count == 0 || negative_count == 0) {
        return std::nullopt;
      }

      Vector3 const positive =
        positive_sum / static_cast<Scalar>(positive_count);
      Vector3 const negative =
        negative_sum / static_cast<Scalar>(negative_count);
      return pad_profile_sample_t {
        .opening = opening,
        .aperture = positive.x() - negative.x(),
        .midpoint = 0.5 * (positive + negative),
      };
    }

    std::vector<pad_profile_sample_t> build_pad_profile(
      gripper_model_t const& gripper) {
      constexpr int sample_count = 65;
      std::vector<pad_profile_sample_t> profile;
      profile.reserve(sample_count);
      for (int i = 0; i < sample_count; ++i) {
        Scalar const t =
          static_cast<Scalar>(i) / static_cast<Scalar>(sample_count - 1);
        Scalar const opening =
          (1.0 - t) * gripper.opening_lower + t * gripper.opening_upper;
        if (auto const sample = measure_pad_profile(gripper, opening)) {
          profile.push_back(*sample);
        }
      }
      std::sort(
        profile.begin(), profile.end(),
        [](pad_profile_sample_t const& lhs, pad_profile_sample_t const& rhs) {
          return lhs.aperture < rhs.aperture;
        });
      return profile;
    }

    std::optional<pad_profile_sample_t> interpolate_pad_profile(
      std::vector<pad_profile_sample_t> const& profile, Scalar desired_aperture,
      Scalar max_target_width) {
      if (profile.empty()) {
        return std::nullopt;
      }
      Scalar max_aperture = profile.back().aperture;
      if (max_target_width > 0.0) {
        max_aperture = std::min(max_aperture, max_target_width);
      }
      if (desired_aperture > max_aperture + 1e-9) {
        return std::nullopt;
      }
      if (desired_aperture <= profile.front().aperture) {
        return profile.front();
      }
      auto const upper = std::lower_bound(
        profile.begin(), profile.end(), desired_aperture,
        [](pad_profile_sample_t const& sample, Scalar aperture) {
          return sample.aperture < aperture;
        });
      if (upper == profile.end()) {
        return profile.back();
      }
      auto const lower = std::prev(upper);
      Scalar const span = upper->aperture - lower->aperture;
      Scalar const t =
        span > 1e-12 ? (desired_aperture - lower->aperture) / span : 0.0;
      return pad_profile_sample_t {
        .opening = (1.0 - t) * lower->opening + t * upper->opening,
        .aperture = desired_aperture,
        .midpoint = (1.0 - t) * lower->midpoint + t * upper->midpoint,
      };
    }

    Matrix3 nominal_parallel_frame(
      pose_t const& frame_from_ref, pose_t const& frame_from_aug,
      Vector3 preferred_axis) {
      Vector3 y_axis = -Vector3::UnitZ();
      if (preferred_axis.squaredNorm() > 1e-18) {
        Vector3 z_axis = preferred_axis.normalized();
        z_axis -= z_axis.dot(y_axis) * y_axis;
        if (z_axis.norm() > 1e-9) {
          z_axis.normalize();
          Vector3 const x_axis = y_axis.cross(z_axis).normalized();
          Matrix3 frame;
          frame.col(0) = x_axis;
          frame.col(1) = y_axis;
          frame.col(2) = z_axis;
          return frame;
        }
      }

      pose_t const ref_from_aug =
        compose(frame_from_ref, inverse(frame_from_aug));
      Vector3 const y_sum =
        Vector3::UnitZ() + ref_from_aug.orientation.toRotationMatrix().col(2);
      if (y_sum.norm() > 1e-6) {
        y_axis = -y_sum.normalized();
      } else {
        y_axis = -Vector3::UnitZ();
      }
      Vector3 x_axis;
      if (y_axis.cross(Vector3::UnitZ()).norm() < 1e-6) {
        x_axis = y_axis.cross(Vector3::UnitY()).normalized();
      } else {
        x_axis = y_axis.cross(Vector3::UnitZ()).normalized();
      }
      Matrix3 frame;
      frame.col(0) = x_axis;
      frame.col(1) = y_axis;
      frame.col(2) = x_axis.cross(y_axis).normalized();
      return frame;
    }

    std::optional<grasp_seed_t> antipodal_seed(
      ConvexSupportEnvelope const& envelope, pose_t const& frame_from_target,
      gripper_model_t const& gripper, seed_parameter_t const& parameter,
      grasp_sampling_config_t const& config) {
      Vector3 const dir = parameter.orientation_sample.normalized();
      Vector3 const positive = envelope.support(dir, frame_from_target).s;
      Vector3 const negative = envelope.support(-dir, frame_from_target).s;
      Scalar const width = (positive - negative).norm();
      if (config.max_target_width > 0.0 && width > config.max_target_width) {
        return std::nullopt;
      }

      Vector3 width_axis = positive - negative;
      width_axis = width_axis.norm() > 1e-9 ? width_axis.normalized() : dir;
      Vector3 approach = -Vector3::UnitZ();
      approach -= approach.dot(width_axis) * width_axis;
      approach = approach.norm() > 1e-6 ? approach.normalized()
                                        : width_axis.unitOrthogonal();
      Scalar const spin =
        parameter.spin + (parameter.flip ? std::numbers::pi_v<Scalar> : 0.0);
      if (spin != 0.0) {
        approach =
          (Eigen::AngleAxis<Scalar> {spin, width_axis} * approach).normalized();
      }

      Matrix3 R;
      R.col(0) = width_axis;
      R.col(1) = approach;
      R.col(2) = width_axis.cross(approach).normalized();
      Scalar const opening =
        std::clamp(-0.05, gripper.opening_lower, gripper.opening_upper);
      return grasp_seed_t {
        .grasp =
          grasp_t {
            .frame_from_grasp =
              pose_t {
                0.5 * (positive + negative) -
                  config.retreat_distance * approach,
                Quaternion {R}},
            .opening = opening,
          },
        .contact_positive = positive,
        .contact_negative = negative,
      };
    }

    std::optional<grasp_seed_t> parallel_seed(
      ConvexSupportEnvelope const& envelope, pose_t const& frame_from_target,
      Matrix3 const& nominal, std::vector<pad_profile_sample_t> const& profile,
      seed_parameter_t const& parameter,
      grasp_sampling_config_t const& config) {
      Scalar const flip =
        parameter.flip ? std::numbers::pi_v<Scalar> : Scalar {0.0};
      Matrix3 const sampled = nominal *
        Eigen::AngleAxis<Scalar> {parameter.spin + flip, Vector3::UnitY()}
          .toRotationMatrix() *
        Eigen::AngleAxis<Scalar> {
          parameter.orientation_sample.x(), Vector3::UnitX()}
          .toRotationMatrix() *
        Eigen::AngleAxis<Scalar> {
          parameter.orientation_sample.y(), Vector3::UnitZ()}
          .toRotationMatrix();

      Vector3 const dir = sampled.col(0).normalized();
      Vector3 const positive = envelope.support(dir, frame_from_target).s;
      Vector3 const negative = envelope.support(-dir, frame_from_target).s;
      Vector3 width_axis = positive - negative;
      Scalar const width = width_axis.norm();
      width_axis = width > 1e-9 ? width_axis / width : dir;

      Vector3 approach = sampled.col(1);
      approach -= approach.dot(width_axis) * width_axis;
      if (approach.norm() <= 1e-9) {
        approach = -Vector3::UnitZ();
        approach -= approach.dot(width_axis) * width_axis;
      }
      approach = approach.norm() > 1e-9 ? approach.normalized()
                                        : width_axis.unitOrthogonal();

      Matrix3 R;
      R.col(0) = width_axis;
      R.col(1) = approach;
      R.col(2) = width_axis.cross(approach).normalized();
      Scalar max_aperture = profile.back().aperture;
      if (config.max_target_width > 0.0) {
        max_aperture = std::min(max_aperture, config.max_target_width);
      }
      if (width > max_aperture + 1e-9) {
        return std::nullopt;
      }
      Scalar const desired_aperture =
        std::min(width + 2.0 * config.aperture_margin, max_aperture);
      auto const pad = interpolate_pad_profile(
        profile, desired_aperture, config.max_target_width);
      if (!pad.has_value()) {
        return std::nullopt;
      }

      Vector3 const midpoint = 0.5 * (positive + negative);
      return grasp_seed_t {
        .grasp =
          grasp_t {
            .frame_from_grasp =
              pose_t {
                midpoint - R * pad->midpoint -
                  config.retreat_distance * approach,
                Quaternion {R}},
            .opening = pad->opening,
          },
        .contact_positive = positive,
        .contact_negative = negative,
      };
    }

    EntityId temporary_entity(SceneView const& scene) {
      auto value = EntityId::invalid_value - 1;
      while (scene.contains(EntityId {value})) {
        --value;
      }
      return EntityId {value};
    }

    Scalar point_clearance(
      SceneView const& scene, EntityId target, Vector3 const& point) {
      EntityId const point_id = temporary_entity(scene);
      auto point_model = std::make_shared<BodyModel>(body_model_config_t {
        .id = BodyModelId {BodyModelId::invalid_value - 1},
        .inertial = std::nullopt,
        .geometries = {point_geometry_config_t {
          .properties =
            geometry_properties_t {
              .id = GeometryId {0},
              .body_from_geometry = {},
              .material = {},
            },
        }},
      });
      std::vector<BodyInstance> bodies;
      bodies.reserve(scene.bodyCount() + 1);
      for (EntityId id : scene.entityIds()) {
        BodyInstance const& source = scene.body(id);
        bodies.emplace_back(body_instance_config_t {
          .id = id,
          .model = source.modelPtr(),
          .frame_from_body = source.frameFromBody(),
          .motion = source.motion(),
          .mobility = source.mobility(),
        });
      }
      bodies.emplace_back(body_instance_config_t {
        .id = point_id,
        .model = std::move(point_model),
        .frame_from_body = pose_t {point, Quaternion::Identity()},
        .motion = {},
        .mobility = mobility_e::kinematic,
      });
      SceneSnapshot const snapshot {scene_snapshot_config_t {
        .frame = scene.frame(),
        .bodies = std::move(bodies),
      }};

      Scalar clearance = std::numeric_limits<Scalar>::infinity();
      for (EntityId id : scene.entityIds()) {
        if (id == target) {
          continue;
        }
        BodyInstance const& obstacle = scene.body(id);
        for (std::size_t i = 0; i < obstacle.model().geometryCount(); ++i) {
          Geometry const& geometry = obstacle.model().geometry(i);
          if (geometry.type() == geometry_type_e::point) {
            continue;
          }
          contact_feature_t const contact = compute_contact(
            snapshot,
            collision_pair_t {
              .first = geometry_instance_id_t {id, geometry.id()},
              .second = geometry_instance_id_t {point_id, GeometryId {0}},
            });
          clearance = std::min(clearance, contact.gap);
        }
      }
      return clearance;
    }

    bool clears_scene(
      phase_scene_t const& phase, grasp_seed_t const& seed, Scalar margin) {
      if (margin <= 0.0) {
        return true;
      }
      return point_clearance(
               phase.scene, phase.target, seed.contact_positive) >= margin &&
        point_clearance(phase.scene, phase.target, seed.contact_negative) >=
        margin;
    }

    bool same_grasp(grasp_t const& lhs, grasp_t const& rhs) {
      constexpr Scalar pos_tol = 1e-3;
      constexpr Scalar rot_tol = 1e-2;
      constexpr Scalar opening_tol = 1e-3;
      return (lhs.frame_from_grasp.position - rhs.frame_from_grasp.position)
               .norm() <= pos_tol &&
        lhs.frame_from_grasp.orientation.angularDistance(
          rhs.frame_from_grasp.orientation) <= rot_tol &&
        std::abs(lhs.opening - rhs.opening) <= opening_tol;
    }

    pose_t phase_grasp_pose(
      phase_scene_t const& ref, phase_scene_t const& phase,
      pose_t const& frame_from_grasp) {
      pose_t const phase_from_ref = compose(
        phase.scene.body(phase.target).frameFromBody(),
        inverse(ref.scene.body(ref.target).frameFromBody()));
      return compose(phase_from_ref, frame_from_grasp);
    }

  }  // namespace

  grasp_seed_result_t generate_grasp_seeds(
    grasp_sampling_problem_t const& problem,
    grasp_sampling_config_t const& config) {
    if (problem.phases.empty()) {
      return invalid_seed_result(
        "empty_sampling_phases",
        "grasp sampling requires at least one target phase");
    }
    if (
      config.max_seeds <= 0 || config.dir_samples <= 0 ||
      config.spin_samples <= 0 || !std::isfinite(config.spin_step) ||
      !std::isfinite(config.retreat_distance) ||
      config.retreat_distance < 0.0 ||
      !std::isfinite(config.max_target_width) ||
      config.max_target_width < 0.0 || !std::isfinite(config.aperture_margin) ||
      config.aperture_margin < 0.0 ||
      !std::isfinite(config.scene_clearance_margin) ||
      config.scene_clearance_margin < 0.0 ||
      !config.preferred_parallel_axis.allFinite()) {
      return invalid_seed_result(
        "invalid_sampling_config", "grasp sampling configuration is invalid");
    }

    FrameId const frame = problem.phases.front().scene.frame();
    for (phase_scene_t const& phase : problem.phases) {
      if (phase.scene.frame() != frame || !phase.scene.contains(phase.target)) {
        return invalid_seed_result(
          "invalid_sampling_phase",
          "all sampling phases must share a frame and contain their target");
      }
    }
    if (
      problem.gripper.opening_offset.size() !=
        static_cast<Eigen::Index>(
          problem.gripper.state.model().degreeOfFreedomCount()) ||
      problem.gripper.opening_direction.size() !=
        problem.gripper.opening_offset.size() ||
      !problem.gripper.opening_offset.allFinite() ||
      !problem.gripper.opening_direction.allFinite() ||
      !std::isfinite(problem.gripper.opening_lower) ||
      !std::isfinite(problem.gripper.opening_upper) ||
      problem.gripper.opening_lower > problem.gripper.opening_upper) {
      return invalid_seed_result(
        "invalid_sampling_gripper", "gripper opening model is invalid");
    }
    if (
      !problem.gripper.root_link.valid() ||
      problem.gripper.state.model().findLink(problem.gripper.root_link) ==
        nullptr ||
      std::find(
        problem.gripper.state.model().roots().begin(),
        problem.gripper.state.model().roots().end(),
        problem.gripper.root_link) ==
        problem.gripper.state.model().roots().end()) {
      return invalid_seed_result(
        "invalid_sampling_gripper_root",
        "gripper sampling requires a valid kinematic root link");
    }

    BodyInstance const& ref_target =
      problem.phases.front().scene.body(problem.phases.front().target);
    std::optional<ConvexSupportEnvelope> envelope;
    try {
      envelope.emplace(ref_target.model());
    } catch (std::invalid_argument const&) {
      return invalid_seed_result(
        "sampling_target_without_dsf",
        "grasp sampling target requires at least one DSF geometry");
    }

    std::vector<pad_profile_sample_t> profile;
    Matrix3 nominal = Matrix3::Identity();
    if (config.strategy == grasp_sampling_strategy_e::parallel_jaw) {
      profile = build_pad_profile(problem.gripper);
      if (profile.empty()) {
        return invalid_seed_result(
          "missing_parallel_pad_profile",
          "parallel-jaw sampling requires physical pad geometry on both sides");
      }
      BodyInstance const& aug_target =
        problem.phases.back().scene.body(problem.phases.back().target);
      nominal = nominal_parallel_frame(
        ref_target.frameFromBody(), aug_target.frameFromBody(),
        config.preferred_parallel_axis);
    }

    grasp_seed_result_t out;
    out.status = solve_status_e::success;
    for (seed_parameter_t const& parameter : seed_parameters(config)) {
      std::optional<grasp_seed_t> seed =
        config.strategy == grasp_sampling_strategy_e::antipodal_surface
        ? antipodal_seed(
            *envelope, ref_target.frameFromBody(), problem.gripper, parameter,
            config)
        : parallel_seed(
            *envelope, ref_target.frameFromBody(), nominal, profile, parameter,
            config);
      if (!seed.has_value()) {
        ++out.rejected_width;
        continue;
      }
      if (!clears_scene(
            problem.phases.front(), *seed, config.scene_clearance_margin)) {
        ++out.rejected_clearance;
        continue;
      }
      out.seeds.push_back(std::move(*seed));
    }
    if (out.seeds.empty()) {
      out.status = solve_status_e::infeasible;
      out.failure = planner_failure_t {
        .code = "no_grasp_seeds",
        .message = "all grasp seeds were rejected by width or clearance",
        .retryable = true,
      };
    }
    return out;
  }

  grasp_result_t sample_grasps(
    grasp_sampling_problem_t const& problem,
    grasp_sampling_config_t const& sampling_config,
    grasp_generation_config_t const& generation_config) {
    grasp_seed_result_t const seeds =
      generate_grasp_seeds(problem, sampling_config);
    if (seeds.status != solve_status_e::success) {
      return grasp_result_t {
        .status = seeds.status,
        .candidates = {},
        .selected_index = std::nullopt,
        .failure = seeds.failure,
      };
    }

    std::vector<grasp_candidate_t> feasible;
    std::vector<grasp_candidate_t> failed;
    for (grasp_seed_t const& seed : seeds.seeds) {
      grasp_result_t result = solve_grasp_pose(
        grasp_problem_t {
          .phases = problem.phases,
          .gripper = problem.gripper,
          .seed = seed.grasp,
        },
        generation_config);
      if (result.candidates.empty()) {
        continue;
      }
      grasp_candidate_t candidate = std::move(result.candidates.front());
      if (result.status == solve_status_e::success) {
        feasible.push_back(std::move(candidate));
      } else {
        failed.push_back(std::move(candidate));
      }
    }

    std::sort(
      feasible.begin(), feasible.end(),
      [](grasp_candidate_t const& lhs, grasp_candidate_t const& rhs) {
        return lhs.score > rhs.score;
      });
    std::vector<grasp_candidate_t> distinct;
    distinct.reserve(feasible.size());
    for (grasp_candidate_t& candidate : feasible) {
      bool const duplicate = std::any_of(
        distinct.begin(), distinct.end(), [&](grasp_candidate_t const& other) {
          return same_grasp(candidate.grasp, other.grasp);
        });
      if (!duplicate) {
        distinct.push_back(std::move(candidate));
      }
    }
    std::size_t const feasible_count = distinct.size();
    distinct.insert(
      distinct.end(), std::make_move_iterator(failed.begin()),
      std::make_move_iterator(failed.end()));
    if (feasible_count == 0) {
      return grasp_result_t {
        .status = solve_status_e::infeasible,
        .candidates = std::move(distinct),
        .selected_index = std::nullopt,
        .failure =
          planner_failure_t {
            .code = "sampled_grasps_infeasible",
            .message = "no sampled grasp satisfied refinement constraints",
            .retryable = true,
          },
      };
    }
    return grasp_result_t {
      .status = solve_status_e::success,
      .candidates = std::move(distinct),
      .selected_index = std::size_t {0},
      .failure = {},
    };
  }

  joint_grasp_result_t sample_joint_grasps(
    joint_grasp_sampling_problem_t const& problem,
    grasp_sampling_config_t const& sampling_config,
    grasp_generation_config_t const& generation_config,
    inverse_kinematics_config_t const& ik_config) {
    if (
      problem.initial_states.size() != problem.grasp.phases.size() ||
      problem.initial_states.empty() || !problem.grasp_link.valid() ||
      !is_valid(problem.link_from_grasp)) {
      return joint_grasp_result_t {
        .status = solve_status_e::invalid_problem,
        .candidates = {},
        .selected_index = std::nullopt,
        .failure =
          planner_failure_t {
            .code = "invalid_joint_sampling_problem",
            .message =
              "joint sampling requires one state per phase and a valid link",
            .retryable = false,
          },
      };
    }

    grasp_seed_result_t const seeds =
      generate_grasp_seeds(problem.grasp, sampling_config);
    if (seeds.status != solve_status_e::success) {
      return joint_grasp_result_t {
        .status = seeds.status,
        .candidates = {},
        .selected_index = std::nullopt,
        .failure = seeds.failure,
      };
    }

    std::vector<joint_grasp_candidate_t> feasible;
    std::vector<joint_grasp_candidate_t> failed;
    phase_scene_t const& ref = problem.grasp.phases.front();
    for (grasp_seed_t const& seed : seeds.seeds) {
      std::vector<KinematicState> states = problem.initial_states;
      planner_failure_t ik_failure;
      bool ik_solved = true;
      for (std::size_t phase = 0; phase < states.size(); ++phase) {
        pose_t const frame_from_grasp = phase_grasp_pose(
          ref, problem.grasp.phases[phase], seed.grasp.frame_from_grasp);
        pose_t const frame_from_link =
          compose(frame_from_grasp, inverse(problem.link_from_grasp));
        inverse_kinematics_config_t local_ik_config = ik_config;
        if (problem.ik_initializer) {
          std::optional<Eigen::VectorXd> const initialized =
            problem.ik_initializer(
              states[phase], problem.grasp_link, frame_from_link);
          if (
            !initialized.has_value() ||
            initialized->size() != states[phase].positions().size() ||
            !initialized->allFinite()) {
            ik_failure = planner_failure_t {
              .code = "joint_sampling_initializer_failed",
              .message =
                "joint grasp IK initializer did not produce a valid seed",
              .retryable = true,
            };
            ik_solved = false;
            break;
          }
          states[phase].setPositions(*initialized);
          local_ik_config.initialization =
            inverse_kinematics_initialization_e::provided;
        }
        inverse_kinematics_result_t const ik = solve_inverse_kinematics(
          inverse_kinematics_problem_t {
            .initial_state = states[phase],
            .link = problem.grasp_link,
            .frame_from_link = frame_from_link,
            .position_only = false,
          },
          local_ik_config);
        if (ik.status != solve_status_e::success) {
          ik_failure = ik.failure;
          ik_solved = false;
          break;
        }
        states[phase].setPositions(ik.positions);
      }
      if (!ik_solved) {
        failed.push_back(joint_grasp_candidate_t {
          .grasp =
            grasp_candidate_t {
              .grasp = seed.grasp,
              .score = 0.0,
              .contacts = {},
              .solver = {},
              .failure = std::move(ik_failure),
            },
          .positions = {},
        });
        continue;
      }

      joint_grasp_result_t result = solve_joint_grasp(
        joint_grasp_problem_t {
          .grasp =
            grasp_problem_t {
              .phases = problem.grasp.phases,
              .gripper = problem.grasp.gripper,
              .seed = seed.grasp,
            },
          .initial_states = std::move(states),
          .grasp_link = problem.grasp_link,
          .link_from_grasp = problem.link_from_grasp,
        },
        generation_config);
      if (result.candidates.empty()) {
        continue;
      }
      joint_grasp_candidate_t candidate = std::move(result.candidates.front());
      if (result.status == solve_status_e::success) {
        feasible.push_back(std::move(candidate));
      } else {
        failed.push_back(std::move(candidate));
      }
    }

    std::sort(
      feasible.begin(), feasible.end(),
      [](
        joint_grasp_candidate_t const& lhs,
        joint_grasp_candidate_t const& rhs) {
        return lhs.grasp.score > rhs.grasp.score;
      });
    std::vector<joint_grasp_candidate_t> distinct;
    distinct.reserve(feasible.size());
    for (joint_grasp_candidate_t& candidate : feasible) {
      bool const duplicate = std::any_of(
        distinct.begin(), distinct.end(),
        [&](joint_grasp_candidate_t const& other) {
          return same_grasp(candidate.grasp.grasp, other.grasp.grasp);
        });
      if (!duplicate) {
        distinct.push_back(std::move(candidate));
      }
    }
    std::size_t const feasible_count = distinct.size();
    distinct.insert(
      distinct.end(), std::make_move_iterator(failed.begin()),
      std::make_move_iterator(failed.end()));
    if (feasible_count == 0) {
      return joint_grasp_result_t {
        .status = solve_status_e::infeasible,
        .candidates = std::move(distinct),
        .selected_index = std::nullopt,
        .failure =
          planner_failure_t {
            .code = "sampled_joint_grasps_infeasible",
            .message = "no sampled grasp satisfied IK and joint refinement",
            .retryable = true,
          },
      };
    }
    return joint_grasp_result_t {
      .status = solve_status_e::success,
      .candidates = std::move(distinct),
      .selected_index = std::size_t {0},
      .failure = {},
    };
  }

}  // namespace stacking_core
