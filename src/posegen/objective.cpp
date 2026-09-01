#include "objective.hpp"

#include <stacking_core/geometry/dsf_vert.hpp>
#include <stacking_core/optimization/truncated_conjugate_gradient.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace stacking_core::posegen_detail {
namespace {

using ground_edge_t = force_graph::edge_t<force_factor_e::ground_contact>;
using pair_edge_t = force_graph::edge_t<force_factor_e::pair_contact>;

Scalar mass(BodyInstance const& body) {
  return body.model().hasInertial() ? body.model().inertial().mass : 1.0;
}

Scalar combine_friction(Geometry const& first, Geometry const& second) {
  Scalar const a = first.material().friction;
  Scalar const b = second.material().friction;
  return 2.0 * a * b / (a + b);
}

Scalar ground_friction(Geometry const& geometry) {
  Scalar const a = geometry.material().friction;
  constexpr Scalar b = 2.0;
  return 2.0 * a * b / (a + b);
}

std::vector<DsfVertGeometry const*> dsf_geometries(
  BodyInstance const& body) {
  std::vector<DsfVertGeometry const*> out;
  for (std::size_t idx = 0; idx < body.model().geometryCount(); ++idx) {
    Geometry const& geometry = body.model().geometry(idx);
    if (geometry.type() == geometry_type_e::dsf_vert) {
      out.push_back(static_cast<DsfVertGeometry const*>(&geometry));
    }
  }
  return out;
}

bool body_support(
  BodyInstance const& body,
  Vector3 const& dir,
  diffable_support_t& support) {
  bool found = false;
  for (DsfVertGeometry const* geometry : dsf_geometries(body)) {
    diffable_support_t candidate = geometry->diffable_support(
      dir, body.frameFromBody());
    if (!found || candidate.h > support.h) {
      support = std::move(candidate);
      found = true;
    }
  }
  return found;
}

Scalar body_support_height(
  BodyInstance const& body,
  Vector3 const& dir) {
  bool found = false;
  Scalar height = 0.0;
  for (DsfVertGeometry const* geometry : dsf_geometries(body)) {
    Scalar const candidate = geometry->support(
      dir, body.frameFromBody()).h;
    if (!found || candidate > height) {
      height = candidate;
      found = true;
    }
  }
  return height;
}

Vector3 body_support_center(BodyInstance const& body) {
  for (DsfVertGeometry const* geometry : dsf_geometries(body)) {
    return body.frameFromGeometry(geometry->id()).position;
  }
  return body.frameFromBody().position;
}

Matrix3 contact_frame(Vector3 const& normal) {
  Vector3 const tangent = normal.unitOrthogonal();
  Matrix3 R;
  R << tangent, normal.cross(tangent), normal;
  return R;
}

Matrix36 wrench_jac(Vector3 const& point, Vector3 const& ref) {
  Matrix36 J;
  J << Matrix3::Identity(), -skew(point - ref);
  return J;
}

Vector3 vex(Matrix3 const& value) {
  return Vector3 {-value(1, 2), value(0, 2), -value(0, 1)};
}

Vector6 cone_pose_grad(
  Vector3 const& force,
  Vector3 const& normal,
  Matrix36 const& d_normal,
  Scalar friction,
  Scalar eps) {
  static_cast<void>(friction);
  Vector3 const tangent = force - normal.dot(force) * normal;
  Scalar const denom = std::sqrt(tangent.squaredNorm() + eps * eps);
  return normal.dot(force) / denom * d_normal.transpose() * force;
}

template <force_factor_e factor>
struct warm_start_t {
  typename force_graph::edge_t<factor>::point_array_t points;
  typename force_graph::edge_t<factor>::point_array_t normals;
  std::array<Vector3, force_graph::edge_t<factor>::cardinality> force;
  std::array<Vector3, force_graph::edge_t<factor>::cardinality>
    consensus_dual;
  std::array<Vector4, force_graph::edge_t<factor>::cardinality> contact_dual;
  std::array<Vector4, force_graph::edge_t<factor>::cardinality> auxiliary;
  Vector3 representative_force = Vector3::Zero();
  Scalar contact_multiplier = 0.0;
  bool used = false;
};

template <force_factor_e factor>
using warm_start_map_t = std::multimap<
  typename force_graph::edge_t<factor>::entity_array_t,
  warm_start_t<factor>>;

template <force_factor_e factor>
warm_start_map_t<factor> capture_warm_start(ForceGraph const& graph) {
  warm_start_map_t<factor> warm;
  for (auto const& [entities, edge] : graph.edges<factor>()) {
    warm_start_t<factor> state;
    state.points = edge.contact_point;
    state.normals = edge.contact_normal;
    state.auxiliary = edge.auxiliary;
    state.representative_force = edge.representative_force;
    state.contact_multiplier = edge.contact_multiplier;
    bool valid = true;
    for (std::size_t idx = 0; idx < edge.cardinality; ++idx) {
      auto const node_iter = graph.nodes().find(entities[idx]);
      if (node_iter == graph.nodes().end()) {
        valid = false;
        break;
      }
      force_graph::node_t const& node = node_iter->second;
      std::size_t const node_idx = edge.node_indices[idx];
      if (node_idx >= node.force_storage.size() ||
          node_idx >= node.consensus_dual_storage.size() ||
          node_idx >= node.contact_dual_storage.size()) {
        valid = false;
        break;
      }
      state.force[idx] = node.force_storage[node_idx];
      state.consensus_dual[idx] = node.consensus_dual_storage[node_idx];
      state.contact_dual[idx] = node.contact_dual_storage[node_idx];
    }
    if (valid) {
      warm.emplace(entities, std::move(state));
    }
  }
  return warm;
}

template <force_factor_e factor>
void apply_warm_start(
  ForceGraph& graph,
  typename force_graph::edge_t<factor>::entity_array_t const& entities,
  force_graph::edge_t<factor>& edge,
  warm_start_map_t<factor>& warm) {
  auto const range = warm.equal_range(entities);
  auto best = range.second;
  Scalar best_dist = std::numeric_limits<Scalar>::infinity();
  for (auto iter = range.first; iter != range.second; ++iter) {
    if (iter->second.used) {
      continue;
    }
    Scalar dist = 0.0;
    for (std::size_t idx = 0; idx < edge.cardinality; ++idx) {
      dist += (edge.contact_point[idx] - iter->second.points[idx])
        .squaredNorm();
      dist += 1e-4 *
        (edge.contact_normal[idx] - iter->second.normals[idx]).squaredNorm();
    }
    if (dist < best_dist) {
      best_dist = dist;
      best = iter;
    }
  }
  constexpr Scalar max_shift = 5e-2;
  if (best == range.second ||
      best_dist > edge.cardinality * max_shift * max_shift) {
    return;
  }
  warm_start_t<factor>& state = best->second;
  for (std::size_t idx = 0; idx < edge.cardinality; ++idx) {
    force_graph::node_t& node = graph.nodes().at(entities[idx]);
    std::size_t const node_idx = edge.node_indices[idx];
    node.force_storage[node_idx] = state.force[idx];
    node.consensus_dual_storage[node_idx] = state.consensus_dual[idx];
    node.contact_dual_storage[node_idx] = state.contact_dual[idx];
  }
  edge.auxiliary = state.auxiliary;
  edge.representative_force = state.representative_force;
  edge.contact_multiplier = state.contact_multiplier;
  state.used = true;
}

struct ground_derivatives_t {
  std::vector<Matrix36> d_jac;
  Vector6 d_gap = Vector6::Zero();
  Matrix36 d_normal = Matrix36::Zero();
};

ground_derivatives_t ground_derivatives(
  diffable_support_t const& support,
  Matrix3 const& contact_from_force) {
  ground_derivatives_t out;
  out.d_jac.resize(6);
  for (int idx = 0; idx < 3; ++idx) {
    Vector3 axis = Vector3::Zero();
    axis[idx] = 1.0;
    Matrix36 d_jac = Matrix36::Zero();
    d_jac.rightCols<3>() = -skew(support.ds_dq.col(idx) - axis);
    out.d_jac[idx] = contact_from_force.transpose() * d_jac;
    d_jac.rightCols<3>() = -skew(support.ds_dq.col(idx + 3));
    out.d_jac[idx + 3] = contact_from_force.transpose() * d_jac;
  }
  out.d_gap = support.ds_dq.row(2).transpose();
  return out;
}

struct pair_derivatives_t {
  std::array<std::vector<Matrix36>, 2> d_jac;
  Vector6 d_gap = Vector6::Zero();
  Matrix36 d_normal = Matrix36::Zero();
};

pair_derivatives_t pair_derivatives(
  diffable_contact_feature_t const& feature,
  Matrix3 const& contact_from_force) {
  pair_derivatives_t out;
  out.d_jac[0].resize(6);
  out.d_jac[1].resize(6);
  for (int idx = 0; idx < 3; ++idx) {
    Vector3 axis = Vector3::Zero();
    axis[idx] = 1.0;
    Matrix36 d_jac = Matrix36::Zero();
    d_jac.rightCols<3>() = -skew(
      feature.d_point_first.col(idx + 6));
    out.d_jac[0][idx] = -contact_from_force.transpose() * d_jac;
    d_jac.rightCols<3>() = -skew(
      feature.d_point_first.col(idx + 9));
    out.d_jac[0][idx + 3] = -contact_from_force.transpose() * d_jac;

    d_jac.rightCols<3>() = -skew(
      feature.d_point_second.col(idx + 6) - axis);
    out.d_jac[1][idx] = contact_from_force.transpose() * d_jac;
    d_jac.rightCols<3>() = -skew(
      feature.d_point_second.col(idx + 9));
    out.d_jac[1][idx + 3] = contact_from_force.transpose() * d_jac;
  }
  out.d_gap = feature.d_gap.rightCols<6>().transpose();
  out.d_normal = feature.d_normal.rightCols<6>();
  return out;
}

}  // namespace

PoseObjective::PoseObjective(
  posegen_config_t const& config,
  posegen_problem_t const& problem)
    : config_(config),
      problem_(problem),
      initial_pose_(problem.scene.body(problem.candidate).frameFromBody()),
      narrow_config_ {
        .max_dir_iters = 1000,
        .dir_tol = 1e-6,
        .tr_radius_init = 1.0,
        .tr_subproblem_tol = 0.1,
        .gain_ratio_lower_thresh = 0.75,
        .gain_ratio_upper_thresh = 0.25,
        .tr_radius_reduction_rate = 0.25,
        .tr_radius_expansion_rate = 2.0,
      },
      target_ids_(problem.targets.begin(), problem.targets.end()),
      boundary_ids_(problem.boundaries.begin(), problem.boundaries.end()),
      graph_(config, problem.candidate) {
  for (EntityId id : problem.scene.entityIds()) {
    if (id != problem.candidate && !target_ids_.contains(id)) {
      scene_ids_.push_back(id);
    }
  }
  cache_scene_contacts();
  compute_scene_components();
}

bool PoseObjective::can_reuse(posegen_problem_t const& problem) const {
  if (problem.candidate != problem_.candidate ||
      problem.targets != problem_.targets ||
      problem.boundaries != problem_.boundaries ||
      problem.scene.frame() != problem_.scene.frame() ||
      problem.scene.bodyCount() != problem_.scene.bodyCount() ||
      !std::ranges::equal(
        problem.scene.entityIds(), problem_.scene.entityIds())) {
    return false;
  }

  for (EntityId id : problem.scene.entityIds()) {
    BodyInstance const& body = problem.scene.body(id);
    BodyInstance const& cached = problem_.scene.body(id);
    if (body.modelPtr().get() != cached.modelPtr().get()) {
      return false;
    }
    if (id == problem.candidate) {
      continue;
    }
    pose_t const& pose = body.frameFromBody();
    pose_t const& cached_pose = cached.frameFromBody();
    if (!pose.position.isApprox(cached_pose.position, 0.0) ||
        !pose.orientation.coeffs().isApprox(
          cached_pose.orientation.coeffs(), 0.0) ||
        body.mobility() != cached.mobility()) {
      return false;
    }
  }
  return true;
}

void PoseObjective::begin_solve(posegen_problem_t const& problem) {
  if (!can_reuse(problem)) {
    throw std::invalid_argument("cannot reuse posegen objective for this scene");
  }
  problem_ = problem;
  initial_pose_ = problem.scene.body(problem.candidate).frameFromBody();
  scene_graph_rebuilds_ = 0;
  scene_graph_reuses_ = 0;
}

std::shared_ptr<SceneSnapshot const> PoseObjective::make_scene(
  pose_t const& candidate_pose) const {
  std::vector<BodyInstance> bodies;
  bodies.reserve(problem_.scene.bodyCount());
  for (EntityId id : problem_.scene.entityIds()) {
    BodyInstance const& body = problem_.scene.body(id);
    bodies.emplace_back(body_instance_config_t {
      .id = id,
      .model = body.modelPtr(),
      .frame_from_body = id == problem_.candidate
        ? candidate_pose
        : body.frameFromBody(),
      .motion = body.motion(),
      .mobility = body.mobility(),
    });
  }
  return std::make_shared<SceneSnapshot>(scene_snapshot_config_t {
    .frame = problem_.scene.frame(),
    .bodies = std::move(bodies),
  });
}

void PoseObjective::cache_scene_contacts() {
  SceneSnapshot const& scene = problem_.scene.snapshot();
  for (EntityId id : scene_ids_) {
    if (boundary_ids_.contains(id)) {
      continue;
    }
    BodyInstance const& body = scene.body(id);
    for (DsfVertGeometry const* geometry : dsf_geometries(body)) {
      support_t const support = geometry->support(
        -Vector3::UnitZ(), body.frameFromBody());
      Scalar const gap = support.s.z() - config_.objective.ground_height;
      if (gap < config_.objective.narrow_phase_scene_tol) {
        scene_ground_contacts_[id].push_back(ground_contact_t {
          .gap = gap,
          .friction = ground_friction(*geometry),
          .point = support.s,
          .normal = Vector3::UnitZ(),
        });
      }
    }
  }

  SceneView const scene_view {problem_.scene.snapshotPtr(), scene_ids_};
  auto const body_pairs = bounding_volume_broad_phase(
    scene_view, config_.objective.narrow_phase_scene_tol);
  auto const geometry_pairs = bounding_volume_middle_phase(
    scene_view, body_pairs, config_.objective.narrow_phase_scene_tol);
  for (collision_pair_t const& pair : geometry_pairs) {
    EntityId const first_id = pair.first.entity;
    EntityId const second_id = pair.second.entity;
    if (boundary_ids_.contains(first_id) &&
        boundary_ids_.contains(second_id)) {
      continue;
    }
    BodyInstance const& first = scene.body(first_id);
    BodyInstance const& second = scene.body(second_id);
    Geometry const& first_geometry =
      first.model().geometry(pair.first.geometry);
    Geometry const& second_geometry =
      second.model().geometry(pair.second.geometry);
    if (first_geometry.type() != geometry_type_e::dsf_vert ||
        second_geometry.type() != geometry_type_e::dsf_vert) {
      continue;
    }
    contact_feature_t const feature = compute_contact(
      scene, pair, narrow_config_);
    if (feature.gap < config_.objective.narrow_phase_scene_tol) {
      scene_pair_contacts_[{first_id, second_id}].push_back(pair_contact_t {
        .gap = feature.gap,
        .friction = combine_friction(first_geometry, second_geometry),
        .point_first = feature.point_first,
        .point_second = feature.point_second,
        .normal = feature.normal,
      });
    }
  }
}

void PoseObjective::compute_scene_components() {
  for (EntityId id : scene_ids_) {
    scene_component_[id] = id;
  }
  auto root = [&](EntityId id) {
    while (scene_component_.at(id) != id) {
      scene_component_[id] = scene_component_.at(scene_component_.at(id));
      id = scene_component_.at(id);
    }
    return id;
  };
  for (auto const& [ids, contacts] : scene_pair_contacts_) {
    if (contacts.empty()) {
      continue;
    }
    EntityId const first_root = root(ids.first);
    EntityId const second_root = root(ids.second);
    if (first_root != second_root) {
      scene_component_[first_root] = second_root;
    }
  }
  for (auto& [id, component] : scene_component_) {
    component = root(id);
  }
}

PoseObjective::hausdorff_result_t PoseObjective::solve_hausdorff(
  BodyInstance const& target,
  BodyInstance const& candidate) const {
  Vector3 dir = body_support_center(candidate) - body_support_center(target);
  if (dir.squaredNorm() < 1e-12) {
    dir = Vector3::UnitZ();
  } else {
    dir.normalize();
  }

  Scalar radius = config_.hausdorff.radius_init;
  diffable_support_t candidate_support;
  diffable_support_t target_support;
  for (int iter = 0; iter < config_.hausdorff.max_iters; ++iter) {
    if (!body_support(candidate, dir, candidate_support) ||
        !body_support(target, dir, target_support)) {
      break;
    }
    Scalar const objective = target_support.h - candidate_support.h;
    Vector3 const grad = target_support.s - candidate_support.s;
    Vector3 const proj_grad = grad - grad.dot(dir) * dir;
    if (proj_grad.norm() < 1e-6) {
      break;
    }
    Eigen::Matrix<Scalar, 3, 2> V_t;
    V_t.col(0) = std::abs(dir.x()) < 1.0
      ? dir.cross(Vector3::UnitX()).normalized()
      : dir.cross(Vector3::UnitY()).normalized();
    V_t.col(1) = dir.cross(V_t.col(0));
    Eigen::Matrix2d const hess =
      V_t.transpose() *
        (target_support.ds_dx - candidate_support.ds_dx) * V_t -
      grad.dot(dir) * Eigen::Matrix2d::Identity();
    Eigen::Vector2d const grad_t = V_t.transpose() * proj_grad;
    auto const [step, boundary] = truncated_conjugate_gradient(
      -grad_t, hess, Scalar {0.1}, radius);
    Vector3 const new_dir = (dir + V_t * step).normalized();
    Scalar const new_objective =
      body_support_height(target, new_dir) -
      body_support_height(candidate, new_dir);
    Scalar const model = objective + grad_t.dot(step) +
      0.5 * step.dot(hess * step);
    Scalar const rho = (objective - new_objective) / (objective - model);
    if (rho > config_.hausdorff.gain_ratio_lower_thresh / 2.0) {
      dir = new_dir;
    }
    if (rho < config_.hausdorff.gain_ratio_lower_thresh) {
      radius *= config_.hausdorff.radius_reduction_rate;
    } else if (rho > config_.hausdorff.gain_ratio_upper_thresh && boundary) {
      radius = std::min(
        config_.hausdorff.radius_expansion_rate * radius, Scalar {1.0});
    }
  }

  body_support(candidate, dir, candidate_support);
  body_support(target, dir, target_support);
  Vector3 const delta = target_support.s - candidate_support.s;
  Matrix3 const P = Matrix3::Identity() - dir * dir.transpose();
  Matrix36 const d_dir =
    (dir * delta.transpose() + dir.dot(delta) * Matrix3::Identity() -
     P * (target_support.ds_dx - candidate_support.ds_dx))
      .inverse() * (-P * candidate_support.ds_dq);

  hausdorff_result_t out;
  out.point_candidate = candidate_support.s;
  out.point_target = target_support.s;
  out.gap = delta.dot(dir);
  out.d_gap = delta.transpose() * d_dir -
    dir.transpose() * candidate_support.ds_dq;
  out.normal = dir;
  return out;
}

PoseObjective::force_result_t PoseObjective::solve_force(
  std::shared_ptr<SceneSnapshot const> const& scene_ptr,
  pose_t const& candidate_pose) {
  SceneSnapshot const& scene = *scene_ptr;
  BodyInstance const& candidate = scene.body(problem_.candidate);

  struct candidate_ground_t {
    DsfVertGeometry const* geometry = nullptr;
    diffable_support_t support;
    Scalar gap = 0.0;
    Scalar friction = 0.0;
  };
  std::vector<candidate_ground_t> candidate_ground;
  for (DsfVertGeometry const* geometry : dsf_geometries(candidate)) {
    diffable_support_t support = geometry->diffable_support(
      -Vector3::UnitZ(), candidate_pose);
    candidate_ground.push_back(candidate_ground_t {
      .geometry = geometry,
      .support = std::move(support),
      .gap = support.s.z() - config_.objective.ground_height,
      .friction = ground_friction(*geometry),
    });
  }

  struct candidate_pair_t {
    EntityId scene_id;
    diffable_contact_feature_t feature;
    Scalar friction = 0.0;
  };
  std::vector<candidate_pair_t> candidate_pairs;
  std::set<EntityId> touched_roots;
  std::vector<EntityId> collision_ids = scene_ids_;
  collision_ids.push_back(problem_.candidate);
  SceneView const collision_view {scene_ptr, std::move(collision_ids)};
  auto const body_pairs = bounding_volume_broad_phase(
    collision_view, config_.objective.narrow_phase_candidate_tol);
  auto const geometry_pairs = bounding_volume_middle_phase(
    collision_view, body_pairs,
    config_.objective.narrow_phase_candidate_tol);
  for (collision_pair_t const& pair : geometry_pairs) {
    if (pair.second.entity != problem_.candidate) {
      continue;
    }
    EntityId const scene_id = pair.first.entity;
    BodyInstance const& scene_body = scene.body(scene_id);
    Geometry const& scene_geometry =
      scene_body.model().geometry(pair.first.geometry);
    Geometry const& candidate_geometry =
      candidate.model().geometry(pair.second.geometry);
    if (scene_geometry.type() != geometry_type_e::dsf_vert ||
        candidate_geometry.type() != geometry_type_e::dsf_vert) {
      continue;
    }
    diffable_contact_feature_t feature = compute_diffable_contact(
      scene, pair, narrow_config_);
    if (feature.gap >= config_.objective.narrow_phase_candidate_tol) {
      continue;
    }
    candidate_pairs.push_back(candidate_pair_t {
      .scene_id = scene_id,
      .feature = std::move(feature),
      .friction = combine_friction(scene_geometry, candidate_geometry),
    });
    touched_roots.insert(scene_component_.at(scene_id));
  }

  std::set<EntityId> reachable;
  for (auto const& [id, component] : scene_component_) {
    if (touched_roots.contains(component)) {
      reachable.insert(id);
    }
  }

  auto ground_warm =
    capture_warm_start<force_factor_e::ground_contact>(graph_);
  auto pair_warm = capture_warm_start<force_factor_e::pair_contact>(graph_);
  bool const reuse_scene_graph =
    scene_graph_ready_ && reachable == graph_reachable_;
  if (reuse_scene_graph) {
    ++scene_graph_reuses_;
    graph_.remove_edges<force_factor_e::ground_contact>(
      ground_edge_t::entity_array_t {problem_.candidate});
    std::set<pair_edge_t::entity_array_t> candidate_edge_ids;
    for (auto const& [entities, edge] :
         graph_.edges<force_factor_e::pair_contact>()) {
      static_cast<void>(edge);
      if (entities[0] == problem_.candidate ||
          entities[1] == problem_.candidate) {
        candidate_edge_ids.insert(entities);
      }
    }
    for (pair_edge_t::entity_array_t const& entities : candidate_edge_ids) {
      graph_.remove_edges<force_factor_e::pair_contact>(entities);
    }
    graph_.add_node(problem_.candidate, mass(candidate));
  } else {
    ++scene_graph_rebuilds_;
    graph_.clear();
    graph_.add_node(problem_.candidate, mass(candidate));
    for (EntityId id : scene_ids_) {
      graph_.add_node(
        id, mass(scene.body(id)), boundary_ids_.contains(id));
    }

    for (EntityId id : scene_ids_) {
      if (!reachable.contains(id)) {
        continue;
      }
      BodyInstance const& body = scene.body(id);
      Vector3 const ref = body.frameFromBody().position;
      if (!boundary_ids_.contains(id)) {
        for (ground_contact_t const& contact : scene_ground_contacts_[id]) {
          Matrix3 const R = contact_frame(contact.normal);
          ground_edge_t::entity_array_t const entities {id};
          ground_edge_t& edge =
            graph_.add_edge<force_factor_e::ground_contact>(
              entities,
              {R.transpose() * wrench_jac(contact.point, ref)},
              {contact.point},
              {-contact.normal},
              contact.gap,
              contact.friction,
              R);
          apply_warm_start(graph_, entities, edge, ground_warm);
        }
      }
    }

    for (auto const& [ids, contacts] : scene_pair_contacts_) {
      if (!reachable.contains(ids.first) || !reachable.contains(ids.second)) {
        continue;
      }
      BodyInstance const& first = scene.body(ids.first);
      BodyInstance const& second = scene.body(ids.second);
      for (pair_contact_t const& contact : contacts) {
        Matrix3 const R = contact_frame(contact.normal);
        pair_edge_t::entity_array_t const entities {ids.first, ids.second};
        pair_edge_t& edge = graph_.add_edge<force_factor_e::pair_contact>(
          entities,
          {-R.transpose() * wrench_jac(
             contact.point_first, first.frameFromBody().position),
           R.transpose() * wrench_jac(
             contact.point_second, second.frameFromBody().position)},
          {contact.point_first, contact.point_second},
          {contact.normal, -contact.normal},
          contact.gap,
          contact.friction,
          R);
        apply_warm_start(graph_, entities, edge, pair_warm);
      }
    }
    scene_graph_ready_ = true;
    graph_reachable_ = reachable;
  }

  for (candidate_ground_t const& contact : candidate_ground) {
    Matrix3 const R = contact_frame(Vector3::UnitZ());
    ground_edge_t::entity_array_t const entities {problem_.candidate};
    ground_edge_t& edge =
      graph_.add_edge<force_factor_e::ground_contact>(
        entities,
        {R.transpose() * wrench_jac(
          contact.support.s, candidate_pose.position)},
        {contact.support.s},
        {-Vector3::UnitZ()},
        contact.gap,
        contact.friction,
        R);
    ground_derivatives_t const derivatives = ground_derivatives(
      contact.support, R);
    edge.d_jac = {derivatives.d_jac};
    edge.d_gap = derivatives.d_gap;
    edge.d_normal = derivatives.d_normal;
    apply_warm_start(graph_, entities, edge, ground_warm);
  }

  for (candidate_pair_t const& contact : candidate_pairs) {
    BodyInstance const& scene_body = scene.body(contact.scene_id);
    diffable_contact_feature_t const& feature = contact.feature;
    Matrix3 const R = contact_frame(feature.normal);
    pair_edge_t::entity_array_t const entities {
      contact.scene_id, problem_.candidate};
    pair_edge_t& edge = graph_.add_edge<force_factor_e::pair_contact>(
      entities,
      {-R.transpose() * wrench_jac(
         feature.point_first, scene_body.frameFromBody().position),
       R.transpose() * wrench_jac(
         feature.point_second, candidate_pose.position)},
      {feature.point_first, feature.point_second},
      {feature.normal, -feature.normal},
      feature.gap,
      contact.friction,
      R);
    pair_derivatives_t const derivatives = pair_derivatives(feature, R);
    edge.d_jac = derivatives.d_jac;
    edge.d_gap = derivatives.d_gap;
    edge.d_normal = derivatives.d_normal;
    apply_warm_start(graph_, entities, edge, pair_warm);
  }

  posegen_force_solver_stats_t const solver = graph_.solve();

  force_result_t out;
  out.solver = solver;
  Vector6 gravity;
  gravity << 0.0, 0.0, -config_.objective.gravity, 0.0, 0.0, 0.0;
  for (auto& [id, node] : graph_.nodes()) {
    if (node.boundary) {
      continue;
    }
    if (node.jac.rows() == 0 || !node.force) {
      out.net_wrench[id] = node.mass * gravity;
      continue;
    }
    Vector6 const wrench =
      node.jac.transpose() * *node.force + node.mass * gravity;
    out.net_wrench[id] = wrench;
    out.c_feq += 0.5 *
      (wrench.transpose() * config_.objective.k_wrench * wrench).value();
    out.c_comp += 0.5 * config_.objective.k_comp *
      (node.force->transpose() * node.gap.asDiagonal() * *node.force).value();
  }
  return out;
}

objective_eval_t PoseObjective::eval(pose_t const& candidate_pose) {
  std::shared_ptr<SceneSnapshot const> const scene = make_scene(candidate_pose);
  BodyInstance const& candidate = scene->body(problem_.candidate);
  force_result_t force = solve_force(scene, candidate_pose);

  objective_eval_t out;
  out.objective = force.c_feq + force.c_comp;
  out.hess = 1e-4 * Matrix6::Identity();
  out.c_feq = force.c_feq;
  out.c_comp = force.c_comp;
  out.net_wrench = std::move(force.net_wrench);
  out.force_solver = force.solver;
  out.scene_graph_rebuilds = scene_graph_rebuilds_;
  out.scene_graph_reuses = scene_graph_reuses_;

  Scalar candidate_mass = 1.0;
  if (auto const iter = graph_.nodes().find(problem_.candidate);
      iter != graph_.nodes().end()) {
    candidate_mass = iter->second.mass;
  }

  posegen_objective_config_t const& config = config_.objective;
  out.objective += candidate_mass * config.k_potential * config.gravity *
    candidate_pose.position.z();
  out.grad[2] = candidate_mass * config.k_potential * config.gravity;

  Vector3 const initial_position = initial_pose_.position;
  out.objective += 0.5 * config.k_xy *
    (candidate_pose.position.head<2>() - initial_position.head<2>())
      .squaredNorm();

  std::array<Scalar, 4> const bounds {
    initial_position.x() - config.w_box / 2.0,
    initial_position.x() + config.w_box / 2.0,
    initial_position.y() - config.w_box / 2.0,
    initial_position.y() + config.w_box / 2.0,
  };
  std::array<Vector3, 4> const normals {
    -Vector3::UnitX(), Vector3::UnitX(),
    -Vector3::UnitY(), Vector3::UnitY(),
  };
  std::array<int, 4> const axes {0, 0, 1, 1};
  for (std::size_t face = 0; face < bounds.size(); ++face) {
    diffable_support_t support;
    if (!body_support(candidate, normals[face], support)) {
      continue;
    }
    Scalar const sign = normals[face][axes[face]];
    Scalar const gap = std::max(
      sign * (support.s[axes[face]] - bounds[face]), Scalar {0.0});
    Vector6 const d_gap =
      sign * support.ds_dq.row(axes[face]).transpose();
    out.objective += 0.5 * config.k_box * gap * gap;
    out.grad += config.k_box * gap * d_gap;
    out.hess += config.k_box * d_gap * d_gap.transpose();
  }

  for (EntityId target_id : target_ids_) {
    hausdorff_result_t const feature = solve_hausdorff(
      scene->body(target_id), candidate);
    Scalar const gap = std::min(
      feature.gap + config.eps_target, Scalar {0.0});
    out.objective += 0.5 * config.k_target * gap * gap;
    if (gap < 0.0) {
      Vector6 const d_gap = feature.d_gap.transpose();
      out.grad += config.k_target * gap * d_gap;
      out.hess += config.k_target * d_gap * d_gap.transpose();
    }
  }

  Matrix3 const R = candidate_pose.orientation.toRotationMatrix();
  Matrix3 const R_init = initial_pose_.orientation.toRotationMatrix();
  Matrix3 const R_diff = R_init.transpose() * R;
  out.objective += 0.5 * config.k_reg * (3.0 - R_diff.trace());
  out.grad.tail<3>() +=
    0.5 * config.k_reg * vex(R_diff - R_diff.transpose());

  diffable_support_t lower_support;
  if (body_support(candidate, -Vector3::UnitZ(), lower_support)) {
    out.objective +=
      config.k_lower * (lower_support.h + candidate_pose.position.z());
    out.grad.tail<3>() -= config.k_lower *
      lower_support.ds_dq.rightCols<3>().transpose() * Vector3::UnitZ();
  }

  for (auto const& [id, node] : graph_.nodes()) {
    static_cast<void>(node);
    out.contact_force[id] = {};
    out.contact_point[id] = {};
    out.contact_normal[id] = {};
  }

  Vector6 gravity;
  gravity << 0.0, 0.0, -config.gravity, 0.0, 0.0, 0.0;
  std::map<EntityId, std::pair<Scalar, Vector6>> min_gap;
  graph_.for_each_factor([&]<force_factor_e factor>() {
    for (auto& [entities, edge] : graph_.edges<factor>()) {
      bool const scene_edge = std::none_of(
        entities.begin(), entities.end(),
        [&](EntityId id) { return id == problem_.candidate; });
      if (!scene_edge) {
        if constexpr (factor == force_factor_e::pair_contact) {
          EntityId const scene_id = entities[0] == problem_.candidate
            ? entities[1]
            : entities[0];
          auto iter = min_gap.find(scene_id);
          if (iter == min_gap.end() || edge.gap < iter->second.first) {
            min_gap[scene_id] = {edge.gap, edge.d_gap};
          }
        }

        Scalar const gap_c = std::min(
          edge.gap - config.eps_gap, Scalar {0.0});
        if (gap_c < 0.0) {
          out.objective += 0.5 * config.k_gap_c * gap_c * gap_c;
          out.grad += config.k_gap_c * gap_c * edge.d_gap;
          out.hess +=
            config.k_gap_c * edge.d_gap * edge.d_gap.transpose();
        }
        out.c_gap += std::max(-edge.gap, Scalar {0.0});

        Scalar const gap_comp = std::max(
          edge.gap - config.eps_comp, Scalar {0.0});
        if (gap_comp > 0.0) {
          Scalar const force_sq = edge.representative_force.squaredNorm();
          out.grad +=
            config.k_comp * force_sq * gap_comp * edge.d_gap;
          out.hess += config.k_comp * force_sq *
            edge.d_gap * edge.d_gap.transpose();
        }

        // diffsim used R.col(-1) here, which is an invalid Eigen index and
        // drops this KKT term in release builds. Column 2 is the contact normal
        // intended by the cone derivative.
        out.grad += edge.contact_multiplier * cone_pose_grad(
          edge.contact_from_force * edge.representative_force,
          edge.contact_from_force.col(2),
          edge.d_normal,
          edge.friction,
          config.eps_cone);

        for (std::size_t idx = 0; idx < edge.cardinality; ++idx) {
          force_graph::node_t const& node = graph_.nodes().at(entities[idx]);
          if (node.boundary) {
            continue;
          }
          Matrix6 d_jac_force;
          for (std::size_t dim = 0; dim < edge.d_jac[idx].size(); ++dim) {
            out.grad[dim] +=
              ((node.jac.transpose() * *node.force + node.mass * gravity)
                 .transpose() *
               config.k_wrench * edge.d_jac[idx][dim].transpose() *
               edge.representative_force)
                .value();
            d_jac_force.col(dim) =
              edge.d_jac[idx][dim].transpose() * edge.representative_force;
          }
          out.hess +=
            d_jac_force.transpose() * config.k_wrench * d_jac_force;
        }
      }

      if (edge.gap < config.narrow_phase_scene_tol) {
        for (std::size_t idx = 0; idx < edge.cardinality; ++idx) {
          Scalar const sign = idx == 0 ? 1.0 : -1.0;
          EntityId const id = entities[idx];
          out.contact_force[id].push_back(
            sign * edge.contact_from_force * *edge.force[idx]);
          out.contact_point[id].push_back(edge.contact_point[idx]);
          out.contact_normal[id].push_back(edge.contact_normal[idx]);
        }
      }
    }
  });

  for (auto const& [id, gap_derivative] : min_gap) {
    static_cast<void>(id);
    Scalar const gap = gap_derivative.first;
    Vector6 const& d_gap = gap_derivative.second;
    out.objective += 0.5 * config.k_gap * gap * gap;
    out.grad += config.k_gap * gap * d_gap;
    out.hess += config.k_gap * d_gap * d_gap.transpose();
  }
  return out;
}

}  // namespace stacking_core::posegen_detail
