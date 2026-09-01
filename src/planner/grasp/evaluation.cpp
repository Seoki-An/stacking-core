#include "evaluation.hpp"
#include "force.hpp"

#include <stacking_core/collision.hpp>
#include <stacking_core/geometry/plane.hpp>

#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <set>
#include <stdexcept>
#include <unordered_set>
#include <utility>

namespace stacking_core::grasp_detail {
  namespace {

    struct hand_body_t {
      BodyInstance body;
      matrix67_t jac = matrix67_t::Zero();
      bool moves_with_opening = false;
    };

    struct pad_accumulator_t {
      Vector3 point_sum = Vector3::Zero();
      Vector3 normal_sum = Vector3::Zero();
      matrix3x7_t d_point_sum = matrix3x7_t::Zero();
      matrix3x7_t d_normal_sum = matrix3x7_t::Zero();
      int count = 0;

      void add(
        Vector3 const& point, Vector3 const& normal, matrix3x7_t const& d_point,
        matrix3x7_t const& d_normal) {
        point_sum += point;
        normal_sum += normal;
        d_point_sum += d_point;
        d_normal_sum += d_normal;
        ++count;
      }

      bool representative(
        Vector3& point, Vector3& normal, matrix3x7_t& d_point,
        matrix3x7_t& d_normal) const {
        if (count == 0) {
          return false;
        }
        Scalar const n = static_cast<Scalar>(count);
        point = point_sum / n;
        d_point = d_point_sum / n;
        Vector3 const mean = normal_sum / n;
        Scalar const norm = mean.norm();
        if (norm <= 1e-12) {
          return false;
        }
        normal = mean / norm;
        d_normal = (Matrix3::Identity() - normal * normal.transpose()) / norm *
          (d_normal_sum / n);
        return true;
      }
    };

    struct scalar_constraint_t {
      Scalar value = 0.0;
      vector7_t jac = vector7_t::Zero();
    };

    struct tooth_contact_t {
      int order = 0;
      Vector3 point = Vector3::Zero();
      Vector3 normal = Vector3::Zero();
      matrix3x7_t d_point = matrix3x7_t::Zero();
      matrix3x7_t d_normal = matrix3x7_t::Zero();
    };

    struct contact_terms_t {
      std::vector<scalar_constraint_t> clearance;
      std::vector<Scalar> contact_gaps;
      std::vector<vector7_t> d_contact_gaps;
      std::vector<Scalar> flatness;
      std::vector<vector7_t> d_flatness;
      pad_accumulator_t left;
      pad_accumulator_t right;
      std::vector<tooth_contact_t> left_teeth;
      std::vector<tooth_contact_t> right_teeth;
      Scalar min_left = std::numeric_limits<Scalar>::infinity();
      Scalar min_right = std::numeric_limits<Scalar>::infinity();
      vector7_t d_min_left = vector7_t::Zero();
      vector7_t d_min_right = vector7_t::Zero();
      std::vector<grasp_contact_t> contacts;
      std::vector<force_contact_t> force_contacts;
      Scalar score = 0.0;
    };

    grasp_contact_geometry_t const* contact_role(
      gripper_model_t const& gripper, geometry_instance_id_t id) {
      auto const iter = std::find_if(
        gripper.contact_geometries.begin(), gripper.contact_geometries.end(),
        [&](grasp_contact_geometry_t const& role) {
          return role.entity == id.entity && role.geometry == id.geometry;
        });
      return iter == gripper.contact_geometries.end() ? nullptr : &*iter;
    }

    matrix67_t grasp_body_jacobian(
      pose_t const& frame_from_grasp, KinematicSnapshot const& snapshot,
      grasp_link_body_t const& binding,
      Eigen::Ref<Eigen::VectorXd const> opening_dir, bool include_opening) {
      pose_t const& frame_from_body = snapshot.frameFromLink(binding.link);
      pose_t const grasp_from_body =
        compose(inverse(frame_from_grasp), frame_from_body);
      matrix67_t J = matrix67_t::Zero();
      J.topLeftCorner<3, 3>() = Matrix3::Identity();
      J.block<3, 3>(0, 3) = -frame_from_grasp.orientation.toRotationMatrix() *
        skew(grasp_from_body.position);
      J.block<3, 3>(3, 3) =
        grasp_from_body.orientation.toRotationMatrix().transpose();

      Matrix6X const& J_link = snapshot.linkJacobian(binding.link);
      if (include_opening && opening_dir.size() != 0) {
        J.topRows<3>().col(6) = J_link.topRows<3>() * opening_dir;
        J.bottomRows<3>().col(6) =
          frame_from_body.orientation.toRotationMatrix().transpose() *
          J_link.bottomRows<3>() * opening_dir;
      }
      return J;
    }

    std::vector<hand_body_t> instantiate_hand(
      gripper_model_t const& gripper, grasp_t const& grasp,
      bool include_opening) {
      KinematicState state = gripper.state;
      Scalar const opening =
        include_opening ? grasp.opening : gripper.opening_upper;
      state.setPositions(
        gripper.opening_offset + opening * gripper.opening_direction);
      state.setFrameFromRoot(
        gripper.root_link,
        compose(grasp.frame_from_grasp, gripper.grasp_from_root));
      KinematicSnapshot const snapshot = forward_kinematics(state);

      std::vector<hand_body_t> out;
      out.reserve(gripper.collision_bodies.size());
      for (grasp_link_body_t const& binding : gripper.collision_bodies) {
        matrix67_t J = grasp_body_jacobian(
          grasp.frame_from_grasp, snapshot, binding, gripper.opening_direction,
          true);
        bool const moves = J.col(6).cwiseAbs().maxCoeff() > 1e-12;
        if (!include_opening) {
          J.col(6).setZero();
        }
        out.push_back(hand_body_t {
          .body = BodyInstance {body_instance_config_t {
            .id = binding.entity,
            .model = binding.body_model,
            .frame_from_body = snapshot.frameFromLink(binding.link),
            .motion = {},
            .mobility = mobility_e::kinematic,
          }},
          .jac = std::move(J),
          .moves_with_opening = moves,
        });
      }
      return out;
    }

    matrix67_t const& body_jacobian(
      std::vector<hand_body_t> const& hand, EntityId entity) {
      auto const iter = std::find_if(
        hand.begin(), hand.end(),
        [&](hand_body_t const& value) { return value.body.id() == entity; });
      if (iter == hand.end()) {
        throw std::logic_error("gripper body Jacobian was not instantiated");
      }
      return iter->jac;
    }

    std::shared_ptr<SceneSnapshot const> combined_snapshot(
      phase_scene_t const& phase, std::vector<hand_body_t> const& hand,
      std::vector<EntityId>& ids) {
      std::vector<BodyInstance> bodies;
      bodies.reserve(phase.scene.bodyCount() + hand.size());
      ids.clear();
      ids.reserve(phase.scene.bodyCount() + hand.size());
      for (EntityId id : phase.scene.entityIds()) {
        BodyInstance const& source = phase.scene.body(id);
        ids.push_back(id);
        bodies.emplace_back(body_instance_config_t {
          .id = source.id(),
          .model = source.modelPtr(),
          .frame_from_body = source.frameFromBody(),
          .motion = source.motion(),
          .mobility = source.mobility(),
        });
      }
      for (hand_body_t const& value : hand) {
        ids.push_back(value.body.id());
        bodies.emplace_back(body_instance_config_t {
          .id = value.body.id(),
          .model = value.body.modelPtr(),
          .frame_from_body = value.body.frameFromBody(),
          .motion = {},
          .mobility = mobility_e::kinematic,
        });
      }
      return std::make_shared<SceneSnapshot>(scene_snapshot_config_t {
        .frame = phase.scene.frame(),
        .bodies = std::move(bodies),
      });
    }

    std::vector<collision_pair_t> geometry_pairs(
      SceneView const& scene, std::vector<hand_body_t> const& hand,
      std::vector<EntityId> const& obstacles) {
      std::vector<collision_body_pair_t> body_pairs;
      body_pairs.reserve(hand.size() * obstacles.size());
      for (hand_body_t const& value : hand) {
        for (EntityId obstacle : obstacles) {
          body_pairs.push_back({value.body.id(), obstacle});
        }
      }
      return brute_force_middle_phase(scene, body_pairs);
    }

    bool is_plane_pair(
      SceneSnapshot const& scene, collision_pair_t const& pair) {
      return scene.body(pair.first.entity)
               .model()
               .geometry(pair.first.geometry)
               .type() == geometry_type_e::plane ||
        scene.body(pair.second.entity)
          .model()
          .geometry(pair.second.geometry)
          .type() == geometry_type_e::plane;
    }

    matrix7_t phase_map(pose_t const& phase_from_ref) {
      matrix7_t map = matrix7_t::Identity();
      map.topLeftCorner<3, 3>() = phase_from_ref.orientation.toRotationMatrix();
      return map;
    }

    grasp_t phase_grasp(
      phase_scene_t const& ref, phase_scene_t const& phase,
      grasp_t const& grasp) {
      pose_t const phase_from_ref = compose(
        phase.scene.body(phase.target).frameFromBody(),
        inverse(ref.scene.body(ref.target).frameFromBody()));
      return grasp_t {
        .frame_from_grasp = compose(phase_from_ref, grasp.frame_from_grasp),
        .opening = grasp.opening,
      };
    }

    void add_obstacle_clearance(
      grasp_problem_t const& problem, grasp_generation_config_t const& config,
      grasp_t const& grasp, contact_terms_t& terms) {
      phase_scene_t const& ref = problem.phases.front();
      for (phase_scene_t const& phase : problem.phases) {
        pose_t const phase_from_ref = compose(
          phase.scene.body(phase.target).frameFromBody(),
          inverse(ref.scene.body(ref.target).frameFromBody()));
        matrix7_t const map = phase_map(phase_from_ref);
        grasp_t const local_grasp = phase_grasp(ref, phase, grasp);
        std::vector<hand_body_t> hand =
          instantiate_hand(problem.gripper, local_grasp, true);
        for (hand_body_t& value : hand) {
          value.jac *= map;
        }

        std::vector<EntityId> obstacles;
        for (EntityId id : phase.scene.entityIds()) {
          if (id != phase.target) {
            obstacles.push_back(id);
          }
        }
        std::vector<EntityId> ids;
        auto snapshot = combined_snapshot(phase, hand, ids);
        SceneView const view {snapshot, ids};
        for (collision_pair_t const& pair :
             geometry_pairs(view, hand, obstacles)) {
          diffable_contact_feature_t const feature =
            compute_diffable_contact(*snapshot, pair);
          vector7_t const d_gap = (feature.d_gap.leftCols<6>() *
                                   body_jacobian(hand, pair.first.entity))
                                    .transpose();
          Scalar const margin = is_plane_pair(*snapshot, pair)
            ? config.plane_separate_margin
            : config.separate_margin;
          terms.clearance.push_back({margin - feature.gap, -d_gap});
        }

        std::vector<hand_body_t> open_hand =
          instantiate_hand(problem.gripper, local_grasp, false);
        for (hand_body_t& value : open_hand) {
          value.jac *= map;
        }
        auto open_snapshot = combined_snapshot(phase, open_hand, ids);
        SceneView const open_view {open_snapshot, ids};
        for (collision_pair_t const& pair :
             geometry_pairs(open_view, open_hand, obstacles)) {
          auto const body_iter = std::find_if(
            open_hand.begin(), open_hand.end(), [&](hand_body_t const& value) {
              return value.body.id() == pair.first.entity;
            });
          if (body_iter == open_hand.end() || !body_iter->moves_with_opening) {
            continue;
          }
          diffable_contact_feature_t const feature =
            compute_diffable_contact(*open_snapshot, pair);
          vector7_t const d_gap =
            (feature.d_gap.leftCols<6>() * body_iter->jac).transpose();
          Scalar const margin = is_plane_pair(*open_snapshot, pair)
            ? config.plane_separate_margin
            : config.separate_margin;
          terms.clearance.push_back({margin - feature.gap, -d_gap});
        }
      }
    }

    void add_target_contacts(
      grasp_problem_t const& problem, grasp_generation_config_t const& config,
      grasp_t const& grasp, contact_terms_t& terms) {
      phase_scene_t const& phase = problem.phases.front();
      std::vector<hand_body_t> const hand =
        instantiate_hand(problem.gripper, grasp, true);
      std::vector<EntityId> ids;
      auto snapshot = combined_snapshot(phase, hand, ids);
      SceneView const view {snapshot, ids};
      std::vector<collision_pair_t> const pairs =
        geometry_pairs(view, hand, {phase.target});

      for (hand_body_t const& hand_body : hand) {
        for (std::size_t geometry_index = 0;
             geometry_index < hand_body.body.model().geometryCount();
             ++geometry_index) {
          Geometry const& geometry =
            hand_body.body.model().geometry(geometry_index);
          collision_pair_t const* min_pair = nullptr;
          diffable_contact_feature_t min_feature;
          for (collision_pair_t const& pair : pairs) {
            if (
              pair.first.entity != hand_body.body.id() ||
              pair.first.geometry != geometry.id()) {
              continue;
            }
            diffable_contact_feature_t const feature =
              compute_diffable_contact(*snapshot, pair);
            if (min_pair == nullptr || feature.gap < min_feature.gap) {
              min_pair = &pair;
              min_feature = feature;
            }
          }
          if (min_pair == nullptr || !std::isfinite(min_feature.gap)) {
            continue;
          }

          matrix67_t const& J = body_jacobian(hand, hand_body.body.id());
          vector7_t const d_gap =
            (min_feature.d_gap.leftCols<6>() * J).transpose();
          terms.clearance.push_back(
            {config.separate_margin - min_feature.gap, -d_gap});

          grasp_contact_geometry_t const* role =
            contact_role(problem.gripper, min_pair->first);
          if (role == nullptr) {
            continue;
          }
          matrix3x7_t const d_point =
            min_feature.d_point_second.leftCols<6>() * J;
          matrix3x7_t const d_normal = min_feature.d_normal.leftCols<6>() * J;
          terms.contact_gaps.push_back(min_feature.gap);
          terms.d_contact_gaps.push_back(d_gap);
          std::size_t const contact_index = terms.contacts.size();
          terms.contacts.push_back(grasp_contact_t {
            .feature = min_feature,
            .force = std::nullopt,
          });
          if (role->contributes_force) {
            terms.force_contacts.push_back(force_contact_t {
              .feature = min_feature,
              .body_jac = J,
              .contact_index = contact_index,
            });
          }

          Vector3 const target_center =
            snapshot->body(min_pair->second.entity)
              .frameFromGeometry(min_pair->second.geometry)
              .position;
          Vector3 const r = min_feature.point_second - target_center;
          terms.flatness.push_back(-r.dot(min_feature.normal));
          terms.d_flatness.push_back(
            -(d_point.transpose() * min_feature.normal +
              d_normal.transpose() * r));

          if (role->type == grasp_contact_type_e::pad) {
            Scalar const side_score = -min_feature.gap *
              std::abs(grasp.frame_from_grasp.orientation.toRotationMatrix()
                         .col(0)
                         .dot(min_feature.normal));
            terms.score += side_score;
            pad_accumulator_t& accumulator =
              role->side == grasp_contact_side_e::left ? terms.left
                                                       : terms.right;
            accumulator.add(
              min_feature.point_second, min_feature.normal, d_point, d_normal);
          } else {
            tooth_contact_t tooth {
              .order = role->order,
              .point = min_feature.point_second,
              .normal = min_feature.normal,
              .d_point = d_point,
              .d_normal = d_normal,
            };
            std::vector<tooth_contact_t>& teeth =
              role->side == grasp_contact_side_e::left ? terms.left_teeth
                                                       : terms.right_teeth;
            teeth.push_back(std::move(tooth));
            terms.score += config.score.teeth_gap * (-min_feature.gap);
          }
          if (role->enforce_contact) {
            Scalar& min_gap = role->side == grasp_contact_side_e::left
              ? terms.min_left
              : terms.min_right;
            vector7_t& d_min_gap = role->side == grasp_contact_side_e::left
              ? terms.d_min_left
              : terms.d_min_right;
            if (min_feature.gap < min_gap) {
              min_gap = min_feature.gap;
              d_min_gap = d_gap;
            }
          }
        }
      }
    }

    matrix3x7_t approach_jacobian(pose_t const& grasp) {
      Matrix3 const R = grasp.orientation.toRotationMatrix();
      matrix3x7_t J = matrix3x7_t::Zero();
      J.col(3) = R.col(2);
      J.col(5) = -R.col(0);
      return J;
    }

  }  // namespace

  bool grasp_evaluation_t::finite() const noexcept {
    return std::isfinite(objective) && std::isfinite(score) &&
      grad.allFinite() && c_ineq.allFinite() && c_eq.allFinite() &&
      jac_ineq.allFinite() && jac_eq.allFinite();
  }

  std::optional<planner_failure_t> validate_grasp_problem(
    grasp_problem_t const& problem, grasp_generation_config_t const& config) {
    auto fail = [](std::string code, std::string message) {
      return planner_failure_t {
        .code = std::move(code),
        .message = std::move(message),
        .retryable = false,
      };
    };
    if (problem.phases.empty()) {
      return fail(
        "empty_phases", "grasp generation requires at least one phase");
    }
    if (
      !is_valid(problem.seed.frame_from_grasp) ||
      !std::isfinite(problem.seed.opening)) {
      return fail("invalid_seed", "grasp seed must be finite");
    }
    if (
      !problem.gripper.root_link.valid() ||
      problem.gripper.state.model().findLink(problem.gripper.root_link) ==
        nullptr) {
      return fail("invalid_gripper_root", "gripper root link is invalid");
    }
    if (
      std::find(
        problem.gripper.state.model().roots().begin(),
        problem.gripper.state.model().roots().end(),
        problem.gripper.root_link) ==
      problem.gripper.state.model().roots().end()) {
      return fail(
        "gripper_root_not_root", "gripper root link must be a kinematic root");
    }
    Eigen::Index const dof = static_cast<Eigen::Index>(
      problem.gripper.state.model().degreeOfFreedomCount());
    if (
      problem.gripper.opening_offset.size() != dof ||
      problem.gripper.opening_direction.size() != dof ||
      !problem.gripper.opening_offset.allFinite() ||
      !problem.gripper.opening_direction.allFinite()) {
      return fail(
        "invalid_opening_map",
        "gripper opening vectors must match its kinematic DoF count");
    }
    if (
      !std::isfinite(problem.gripper.opening_lower) ||
      !std::isfinite(problem.gripper.opening_upper) ||
      problem.gripper.opening_lower > problem.gripper.opening_upper) {
      return fail(
        "invalid_opening_limits", "gripper opening limits are invalid");
    }
    bool has_left = false;
    bool has_right = false;
    bool constrains_left = false;
    bool constrains_right = false;
    bool contributes_force = false;
    std::unordered_set<EntityId> hand_ids;
    for (grasp_link_body_t const& body : problem.gripper.collision_bodies) {
      if (
        !body.entity.valid() || body.body_model == nullptr ||
        problem.gripper.state.model().findLink(body.link) == nullptr ||
        !hand_ids.emplace(body.entity).second) {
        return fail(
          "invalid_gripper_body",
          "gripper collision bodies require unique IDs, models, and links");
      }
      for (std::size_t i = 0; i < body.body_model->geometryCount(); ++i) {
        if (body.body_model->geometry(i).type() != geometry_type_e::dsf_vert) {
          return fail(
            "unsupported_gripper_geometry",
            "grasp collision bodies currently require DSF geometries");
        }
      }
    }
    std::set<geometry_instance_id_t> contact_ids;
    for (grasp_contact_geometry_t const& role :
         problem.gripper.contact_geometries) {
      auto const body = std::find_if(
        problem.gripper.collision_bodies.begin(),
        problem.gripper.collision_bodies.end(),
        [&](grasp_link_body_t const& value) {
          return value.entity == role.entity;
        });
      if (
        body == problem.gripper.collision_bodies.end() ||
        body->body_model->findGeometry(role.geometry) == nullptr ||
        !contact_ids
           .emplace(geometry_instance_id_t {role.entity, role.geometry})
           .second) {
        return fail(
          "invalid_contact_geometry",
          "each gripper contact role must identify a collision geometry");
      }
      has_left |= role.side == grasp_contact_side_e::left;
      has_right |= role.side == grasp_contact_side_e::right;
      constrains_left |=
        role.side == grasp_contact_side_e::left && role.enforce_contact;
      constrains_right |=
        role.side == grasp_contact_side_e::right && role.enforce_contact;
      contributes_force |= role.contributes_force;
    }
    if (!has_left || !has_right) {
      return fail(
        "missing_contact_side",
        "gripper requires at least one left and one right contact geometry");
    }
    if (!constrains_left || !constrains_right) {
      return fail(
        "missing_contact_constraint",
        "each gripper side requires a contact-enforcing geometry");
    }
    if (config.force.enabled && !contributes_force) {
      return fail(
        "missing_force_contact",
        "force refinement requires a force-contributing contact geometry");
    }
    FrameId const frame = problem.phases.front().scene.frame();
    for (phase_scene_t const& phase : problem.phases) {
      if (phase.scene.frame() != frame || !phase.scene.contains(phase.target)) {
        return fail(
          "invalid_phase",
          "all grasp phases must share a frame and contain their target");
      }
      for (EntityId scene_id : phase.scene.entityIds()) {
        BodyInstance const& body = phase.scene.body(scene_id);
        for (std::size_t i = 0; i < body.model().geometryCount(); ++i) {
          geometry_type_e const type = body.model().geometry(i).type();
          if (
            type == geometry_type_e::point ||
            (scene_id == phase.target && type != geometry_type_e::dsf_vert)) {
            return fail(
              "unsupported_grasp_scene_geometry",
              "grasp scenes require DSF targets and DSF or plane obstacles");
          }
        }
      }
      for (EntityId id : hand_ids) {
        if (phase.scene.contains(id)) {
          return fail(
            "gripper_entity_conflict",
            "gripper entity IDs must be outside every phase scene");
        }
      }
    }
    auto const& tr = config.trust_region;
    auto const& alm = config.alm;
    auto const& force = config.force;
    if (
      tr.max_iters <= 0 || tr.subproblem_tol <= 0.0 || tr.radius_init <= 0.0 ||
      tr.radius_max < tr.radius_init || tr.radius_reduction <= 0.0 ||
      tr.radius_reduction >= 1.0 || tr.radius_expansion <= 1.0 ||
      tr.step_tol <= 0.0 || alm.max_iters <= 0 || alm.inequality_tol < 0.0 ||
      alm.equality_tol < 0.0 || alm.inequality_penalty_init <= 0.0 ||
      alm.equality_penalty_init <= 0.0 ||
      alm.inequality_penalty_increase <= 1.0 ||
      alm.equality_penalty_increase <= 1.0 ||
      !std::isfinite(force.weight.wrench) || force.weight.wrench < 0.0 ||
      !std::isfinite(force.weight.complementarity) ||
      force.weight.complementarity < 0.0 || !std::isfinite(force.weight.cone) ||
      force.weight.cone < 0.0 || !std::isfinite(force.weight.moment) ||
      force.weight.moment < 0.0 || !std::isfinite(force.friction) ||
      force.friction < 0.0 || !std::isfinite(force.wrench_scale) ||
      force.wrench_scale < 0.0 || !std::isfinite(force.damping) ||
      force.damping <= 0.0) {
      return fail("invalid_config", "grasp solver configuration is invalid");
    }
    return std::nullopt;
  }

  grasp_evaluation_t evaluate_grasp(
    grasp_problem_t const& problem, grasp_generation_config_t const& config,
    grasp_t const& grasp) {
    contact_terms_t terms;
    add_obstacle_clearance(problem, config, grasp, terms);
    add_target_contacts(problem, config, grasp, terms);

    grasp_evaluation_t out;
    out.contacts = std::move(terms.contacts);
    Vector3 s_left;
    Vector3 s_right;
    Vector3 u_left;
    Vector3 u_right;
    matrix3x7_t ds_left;
    matrix3x7_t ds_right;
    matrix3x7_t du_left;
    matrix3x7_t du_right;
    bool const contacts_valid =
      terms.left.representative(s_left, u_left, ds_left, du_left) &&
      terms.right.representative(s_right, u_right, ds_right, du_right);

    Scalar cost_antipodal_normal = 0.0;
    Scalar cost_antipodal_position = 0.0;
    vector7_t d_antipodal_normal = vector7_t::Zero();
    vector7_t d_antipodal_position = vector7_t::Zero();
    if (contacts_valid) {
      Vector3 const delta = s_left - s_right;
      Scalar const norm = delta.norm();
      if (norm > 1e-12) {
        Vector3 const dir = delta / norm;
        matrix3x7_t const d_dir =
          (Matrix3::Identity() - dir * dir.transpose()) / norm *
          (ds_left - ds_right);
        cost_antipodal_normal = u_left.dot(u_right);
        cost_antipodal_position = (u_left - u_right).dot(dir);
        d_antipodal_normal =
          du_right.transpose() * u_left + du_left.transpose() * u_right;
        d_antipodal_position = d_dir.transpose() * (u_left - u_right) +
          (du_left - du_right).transpose() * dir;
      }
    }

    auto by_order = [](tooth_contact_t const& lhs, tooth_contact_t const& rhs) {
      return lhs.order < rhs.order;
    };
    std::sort(terms.left_teeth.begin(), terms.left_teeth.end(), by_order);
    std::sort(terms.right_teeth.begin(), terms.right_teeth.end(), by_order);
    Scalar cost_teeth_fit = 0.0;
    vector7_t d_teeth_fit = vector7_t::Zero();
    std::size_t const teeth_count =
      std::min(terms.left_teeth.size(), terms.right_teeth.size());
    for (std::size_t i = 1; i < teeth_count; ++i) {
      tooth_contact_t const& left_prev = terms.left_teeth[i - 1];
      tooth_contact_t const& left = terms.left_teeth[i];
      tooth_contact_t const& right_prev = terms.right_teeth[i - 1];
      tooth_contact_t const& right = terms.right_teeth[i];
      cost_teeth_fit +=
        (left.point - left_prev.point).dot(left.normal - left_prev.normal) +
        (right.point - right_prev.point).dot(right.normal - right_prev.normal);
      d_teeth_fit += (left.d_point - left_prev.d_point).transpose() *
          (left.normal - left_prev.normal) +
        (left.d_normal - left_prev.d_normal).transpose() *
          (left.point - left_prev.point) +
        (right.d_point - right_prev.d_point).transpose() *
          (right.normal - right_prev.normal) +
        (right.d_normal - right_prev.d_normal).transpose() *
          (right.point - right_prev.point);
    }

    BodyInstance const& target =
      problem.phases.front().scene.body(problem.phases.front().target);
    Vector3 const r =
      grasp.frame_from_grasp.position - target.frameFromBody().position;
    matrix3x7_t d_r = matrix3x7_t::Zero();
    d_r.leftCols<3>() = Matrix3::Identity();
    Matrix3 const R = grasp.frame_from_grasp.orientation.toRotationMatrix();
    Vector3 const approach = R.col(1);
    matrix3x7_t const d_approach = approach_jacobian(grasp.frame_from_grasp);
    Scalar const cost_enclosure = r.dot(approach);
    vector7_t const d_enclosure =
      d_r.transpose() * approach + d_approach.transpose() * r;
    Vector3 const radial = r.cross(approach);
    Scalar const cost_radial = radial.norm();
    vector7_t d_radial = vector7_t::Zero();
    if (cost_radial > 1e-12) {
      d_radial = (skew(r) * d_approach - skew(approach) * d_r).transpose() *
        radial / cost_radial;
    }
    Scalar const cost_center = r.squaredNorm();
    vector7_t d_center = vector7_t::Zero();
    d_center.head<3>() = 2.0 * r;

    Scalar cost_align = 0.0;
    Scalar cost_teeth_align = 0.0;
    vector7_t d_align = vector7_t::Zero();
    vector7_t d_teeth_align = vector7_t::Zero();
    phase_scene_t const& ref = problem.phases.front();
    for (phase_scene_t const& phase : problem.phases) {
      pose_t const phase_from_ref = compose(
        phase.scene.body(phase.target).frameFromBody(),
        inverse(ref.scene.body(ref.target).frameFromBody()));
      Vector3 const local_approach = phase_from_ref.orientation * approach;
      matrix3x7_t const d_local_approach =
        phase_from_ref.orientation.toRotationMatrix() * d_approach;
      for (EntityId id : phase.scene.entityIds()) {
        BodyInstance const& body = phase.scene.body(id);
        for (std::size_t index = 0; index < body.model().geometryCount();
             ++index) {
          Geometry const& geometry = body.model().geometry(index);
          if (geometry.type() != geometry_type_e::plane) {
            continue;
          }
          Vector3 const normal =
            static_cast<PlaneGeometry const&>(geometry).normal(
              body.frameFromBody());
          cost_align += local_approach.dot(normal);
          d_align += d_local_approach.transpose() * normal;
          Vector3 const normal_ref =
            phase_from_ref.orientation.conjugate() * normal;
          for (tooth_contact_t const& tooth : terms.left_teeth) {
            cost_teeth_align -= tooth.normal.dot(normal_ref);
            d_teeth_align -= tooth.d_normal.transpose() * normal_ref;
          }
          for (tooth_contact_t const& tooth : terms.right_teeth) {
            cost_teeth_align -= tooth.normal.dot(normal_ref);
            d_teeth_align -= tooth.d_normal.transpose() * normal_ref;
          }
        }
      }
    }

    Scalar cost_contact = 0.0;
    vector7_t d_contact = vector7_t::Zero();
    for (std::size_t i = 0; i < terms.contact_gaps.size(); ++i) {
      cost_contact += 0.5 * terms.contact_gaps[i] * terms.contact_gaps[i];
      d_contact += terms.contact_gaps[i] * terms.d_contact_gaps[i];
    }
    Scalar cost_flatness = 0.0;
    vector7_t d_flatness = vector7_t::Zero();
    for (std::size_t i = 0; i < terms.flatness.size(); ++i) {
      cost_flatness += terms.flatness[i];
      d_flatness += terms.d_flatness[i];
    }

    auto const& weight = config.cost;
    out.objective = weight.antipodal_normal * cost_antipodal_normal +
      weight.antipodal_position * cost_antipodal_position +
      weight.align * cost_align + weight.enclosure * cost_enclosure +
      weight.radial_distance * cost_radial +
      weight.center_distance * cost_center + weight.contact * cost_contact +
      weight.teeth_fit * cost_teeth_fit +
      weight.teeth_align * cost_teeth_align + weight.flatness * cost_flatness;
    out.grad = weight.antipodal_normal * d_antipodal_normal +
      weight.antipodal_position * d_antipodal_position +
      weight.align * d_align + weight.enclosure * d_enclosure +
      weight.radial_distance * d_radial + weight.center_distance * d_center +
      weight.contact * d_contact + weight.teeth_fit * d_teeth_fit +
      weight.teeth_align * d_teeth_align + weight.flatness * d_flatness;

    if (config.force.enabled) {
      force_refinement_t const force = refine_contact_forces(
        terms.force_contacts, target.frameFromBody().position, config.force);
      out.objective += force.cost;
      out.grad += force.grad;
      for (std::size_t i = 0; i < terms.force_contacts.size(); ++i) {
        out.contacts[terms.force_contacts[i].contact_index].force =
          force.contact_forces[i];
      }
    }

    auto const& score_weight = config.score;
    out.score = terms.score - score_weight.center_distance * cost_center -
      score_weight.align * cost_align -
      score_weight.enclosure * cost_enclosure -
      score_weight.radial_distance * cost_radial -
      score_weight.teeth_fit * cost_teeth_fit -
      score_weight.teeth_align * cost_teeth_align;

    Eigen::Index const n_clearance =
      static_cast<Eigen::Index>(terms.clearance.size());
    out.c_ineq = Eigen::VectorXd::Zero(n_clearance + 4);
    out.jac_ineq = matrix_x7_t::Zero(n_clearance + 4, 7);
    for (Eigen::Index i = 0; i < n_clearance; ++i) {
      out.c_ineq(i) = terms.clearance[static_cast<std::size_t>(i)].value;
      out.jac_ineq.row(i) =
        terms.clearance[static_cast<std::size_t>(i)].jac.transpose();
    }
    out.c_ineq(n_clearance) = terms.min_left - config.contact_margin;
    out.c_ineq(n_clearance + 1) = terms.min_right - config.contact_margin;
    out.c_ineq(n_clearance + 2) = grasp.opening - problem.gripper.opening_upper;
    out.c_ineq(n_clearance + 3) =
      -grasp.opening + problem.gripper.opening_lower;
    out.jac_ineq.row(n_clearance) = terms.d_min_left.transpose();
    out.jac_ineq.row(n_clearance + 1) = terms.d_min_right.transpose();
    out.jac_ineq(n_clearance + 2, 6) = 1.0;
    out.jac_ineq(n_clearance + 3, 6) = -1.0;
    out.c_eq = Eigen::VectorXd::Zero(1);
    out.jac_eq = matrix_x7_t::Zero(1, 7);
    return out;
  }

  grasp_t apply_grasp_step(
    grasp_t const& grasp, Eigen::Ref<vector7_t const> step) {
    grasp_t out = grasp;
    out.frame_from_grasp.position += step.head<3>();
    Scalar const angle = step.segment<3>(3).norm();
    Quaternion delta = Quaternion::Identity();
    if (angle > 0.0) {
      delta = Quaternion {
        Eigen::AngleAxis<Scalar> {angle, step.segment<3>(3) / angle}};
    }
    out.frame_from_grasp.orientation =
      normalized(grasp.frame_from_grasp.orientation * delta);
    out.opening += step(6);
    return out;
  }

  Matrix6X body_pose_jacobian(
    KinematicSnapshot const& snapshot, LinkId link,
    pose_t const& link_from_body) {
    Matrix6X const& J_link = snapshot.linkJacobian(link);
    pose_t const& frame_from_link = snapshot.frameFromLink(link);
    pose_t const frame_from_body = compose(frame_from_link, link_from_body);
    Vector3 const offset =
      frame_from_link.orientation * link_from_body.position;
    Matrix6X J = J_link;
    J.topRows<3>() -= skew(offset) * J_link.bottomRows<3>();
    J.bottomRows<3>() =
      frame_from_body.orientation.toRotationMatrix().transpose() *
      J_link.bottomRows<3>();
    return J;
  }

  std::vector<grasp_dynamics_contact_t> evaluate_grasp_dynamics_contacts(
    grasp_problem_t const& problem, grasp_t const& grasp,
    grasp_dynamics_config_t const& config) {
    if (problem.phases.empty()) {
      return {};
    }
    phase_scene_t const& phase = problem.phases.front();
    std::vector<grasp_dynamics_contact_t> out;

    auto append_contacts =
      [&](std::vector<hand_body_t> const& hand, bool open_obstacles_only) {
        std::vector<EntityId> ids;
        auto snapshot = combined_snapshot(phase, hand, ids);
        SceneView const view {snapshot, ids};
        std::vector<EntityId> obstacles(
          phase.scene.entityIds().begin(), phase.scene.entityIds().end());
        for (collision_pair_t const& pair :
             geometry_pairs(view, hand, obstacles)) {
          bool const target = pair.second.entity == phase.target;
          if (open_obstacles_only && target) {
            continue;
          }
          auto const body_iter = std::find_if(
            hand.begin(), hand.end(), [&](hand_body_t const& value) {
              return value.body.id() == pair.first.entity;
            });
          if (
            body_iter == hand.end() ||
            (open_obstacles_only && !body_iter->moves_with_opening)) {
            continue;
          }
          diffable_contact_feature_t const feature =
            compute_diffable_contact(*snapshot, pair);
          Scalar const margin = target
            ? config.target_margin
            : (is_plane_pair(*snapshot, pair) ? config.plane_margin
                                              : config.obstacle_margin);
          if (!std::isfinite(feature.gap) || feature.gap > margin) {
            continue;
          }
          grasp_contact_geometry_t const* role =
            target ? contact_role(problem.gripper, pair.first) : nullptr;
          matrix67_t const& J = body_iter->jac;
          out.push_back(grasp_dynamics_contact_t {
            .feature = feature,
            .d_point_first = feature.d_point_first.leftCols<6>() * J,
            .d_gap = (feature.d_gap.leftCols<6>() * J).transpose(),
            .target_contact = role != nullptr,
            .side = role == nullptr
              ? std::nullopt
              : std::optional<grasp_contact_side_e> {role->side},
          });
        }
      };

    append_contacts(instantiate_hand(problem.gripper, grasp, true), false);
    append_contacts(instantiate_hand(problem.gripper, grasp, false), true);
    return out;
  }

}  // namespace stacking_core::grasp_detail
