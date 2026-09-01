#include <stacking_core/optimization.hpp>
#include <stacking_core/planner/stable_pose.hpp>

#include <Eigen/Eigenvalues>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <memory>
#include <numeric>
#include <optional>
#include <stdexcept>
#include <utility>
#include <vector>

namespace stacking_core {
  namespace {

    using Matrix32 = Eigen::Matrix<Scalar, 3, 2>;
    using Matrix2 = Eigen::Matrix<Scalar, 2, 2>;
    using Vector2 = Eigen::Matrix<Scalar, 2, 1>;

    struct equilibrium_t {
      Vector3 dir = Vector3::UnitZ();
      Scalar grad_norm = 0.0;
      int iters = 0;
    };

    struct envelope_t {
      std::unique_ptr<DsfVertGeometry> geometry;
      Vector3 centroid_body;
    };

    Matrix32 tangent_basis(Vector3 dir) {
      dir.normalize();
      Vector3 x;
      if (dir.x() != 0.0 || dir.y() != 0.0) {
        x = Vector3 {-dir.y(), dir.x(), 0.0};
      } else {
        x = Vector3 {0.0, -dir.z(), dir.y()};
      }
      x.normalize();
      Matrix32 V_t;
      V_t.col(0) = x;
      V_t.col(1) = dir.cross(x);
      return V_t;
    }

    std::optional<envelope_t> make_envelope(BodyModel const& body) {
      std::vector<DsfVertGeometry const*> geometries;
      Eigen::Index node_count = 0;
      int sharpness = 2;
      Scalar epsilon = std::numeric_limits<Scalar>::epsilon();
      for (std::size_t i = 0; i < body.geometryCount(); ++i) {
        Geometry const& geometry = body.geometry(i);
        if (geometry.type() != geometry_type_e::dsf_vert) {
          continue;
        }
        auto const& dsf = static_cast<DsfVertGeometry const&>(geometry);
        if (geometries.empty()) {
          epsilon = dsf.epsilon();
        }
        geometries.push_back(&dsf);
        node_count += dsf.nodes().cols();
        sharpness = std::max(sharpness, dsf.sharpness());
      }
      if (geometries.empty() || node_count == 0) {
        return std::nullopt;
      }

      Matrix3X nodes(3, node_count);
      Eigen::Index column = 0;
      for (DsfVertGeometry const* geometry : geometries) {
        for (Eigen::Index i = 0; i < geometry->nodes().cols(); ++i) {
          nodes.col(column++) = transform_point(
            geometry->bodyFromGeometry(), geometry->nodes().col(i));
        }
      }
      Vector3 const centroid = nodes.rowwise().mean();
      auto envelope =
        std::make_unique<DsfVertGeometry>(dsf_vert_geometry_config_t {
          .properties =
            geometry_properties_t {
              .id = GeometryId {0},
              .body_from_geometry = pose_t {},
              .material = material_t {},
            },
          .nodes = std::move(nodes),
          .sharpness = sharpness,
          .epsilon = epsilon,
        });
      return envelope_t {
        .geometry = std::move(envelope),
        .centroid_body = centroid,
      };
    }

    diffable_support_t support_about(
      DsfVertGeometry const& geometry, Vector3 const& dir,
      Vector3 const& com_body) {
      return geometry.diffable_support(
        dir, pose_t {-com_body, Quaternion::Identity()});
    }

    Scalar equilibrium_residual(
      DsfVertGeometry const& geometry, Vector3 const& dir,
      Vector3 const& com_body) {
      diffable_support_t const support = support_about(geometry, dir, com_body);
      Matrix32 const V_t = tangent_basis(dir);
      return (V_t.transpose() * (Matrix3::Identity() - dir * dir.transpose()) *
              support.s)
        .norm();
    }

    equilibrium_t solve_equilibrium(
      Vector3 const& initial_dir, DsfVertGeometry const& geometry,
      Vector3 const& com_body) {
      constexpr int max_iters = 50;
      constexpr Scalar grad_tol = 1e-8;
      constexpr Scalar step_tol = 1e-12;
      constexpr Scalar radius_max = 0.6;
      constexpr Scalar gain_lower = 0.25;
      constexpr Scalar gain_upper = 0.75;
      constexpr Scalar radius_reduction = 0.25;
      constexpr Scalar radius_expansion = 2.0;
      constexpr Scalar subproblem_tol = 0.1;

      Vector3 dir = initial_dir;
      Scalar radius = radius_max / 8.0;
      int iters = 0;
      for (int iter = 0; iter < max_iters; ++iter) {
        diffable_support_t const support =
          support_about(geometry, dir, com_body);
        Matrix32 const V_t = tangent_basis(dir);
        Vector2 const grad = V_t.transpose() *
          (Matrix3::Identity() - dir * dir.transpose()) * support.s;
        Matrix2 const hess = V_t.transpose() *
          (support.ds_dx - dir.dot(support.s) * Matrix3::Identity()) * V_t;
        auto const [step, on_boundary] =
          truncated_conjugate_gradient(-grad, hess, subproblem_tol, radius);
        Vector3 const next_dir = (dir + V_t * step).normalized();
        diffable_support_t const next_support =
          support_about(geometry, next_dir, com_body);
        Scalar const model_height =
          support.h + grad.dot(step) + 0.5 * step.dot(hess * step);
        Scalar const gain =
          (support.h - next_support.h) / (support.h - model_height);

        if (gain > gain_lower / 2.0) {
          dir = next_dir;
        }
        if (gain < gain_lower) {
          radius *= radius_reduction;
        } else if (gain > gain_upper && on_boundary) {
          radius = std::min(radius * radius_expansion, radius_max);
        }
        iters = iter + 1;
        if (step.norm() < step_tol || grad.norm() < grad_tol) {
          break;
        }
      }
      return equilibrium_t {
        .dir = dir,
        .grad_norm = equilibrium_residual(geometry, dir, com_body),
        .iters = iters,
      };
    }

    std::pair<std::vector<Vector3>, std::vector<int>> distinct_dirs(
      std::vector<Vector3> const& dirs) {
      constexpr Scalar similarity_min = 0.9;
      std::vector<Vector3> distinct;
      std::vector<int> mapping(dirs.size());
      for (std::size_t i = 0; i < dirs.size(); ++i) {
        bool found = false;
        for (std::size_t j = 0; j < distinct.size(); ++j) {
          if (dirs[i].dot(distinct[j]) > similarity_min) {
            mapping[i] = static_cast<int>(j);
            found = true;
            break;
          }
        }
        if (!found) {
          distinct.push_back(dirs[i]);
          mapping[i] = static_cast<int>(distinct.size() - 1);
        }
      }
      return {std::move(distinct), std::move(mapping)};
    }

    Scalar min_eigenvalue(
      Vector3 const& dir, DsfVertGeometry const& geometry,
      Vector3 const& com_body) {
      diffable_support_t const support = support_about(geometry, dir, com_body);
      Matrix32 const V_t = tangent_basis(dir);
      Matrix2 const hess = V_t.transpose() *
        (support.ds_dx - dir.dot(support.s) * Matrix3::Identity()) * V_t;
      Eigen::SelfAdjointEigenSolver<Matrix2> const solver {hess};
      return solver.eigenvalues().minCoeff();
    }

    std::vector<Vector3> sphere_samples(int level) {
      int const side_count = 1 << level;
      float const side_length = 1.0F / static_cast<float>(side_count);
      std::vector<Vector3> samples;
      auto insert = [&](Scalar x, Scalar y, Scalar z) {
        Vector3 dir {
          static_cast<float>(x), static_cast<float>(y), static_cast<float>(z)};
        samples.push_back(dir.normalized());
      };

      for (int iy = -side_count; iy <= 0; ++iy) {
        int const from = -side_count + std::abs(iy);
        int const to = std::abs(from);
        for (int ix = from; ix <= to; ++ix) {
          Scalar const x = ix * side_length;
          Scalar const y = iy * side_length;
          Scalar z = x + y + 1.0;
          if (ix > 0) {
            z = -x + y + 1.0;
          }
          insert(x, z, y);
        }
        for (int ix = from + 1; ix <= to - 1; ++ix) {
          Scalar const x = ix * side_length;
          Scalar const y = iy * side_length;
          Scalar z = -x - y - 1.0;
          if (ix > 0) {
            z = x - y - 1.0;
          }
          insert(x, z, y);
        }
      }
      for (int iy = 1; iy <= side_count; ++iy) {
        int const from = -side_count + std::abs(iy);
        int const to = std::abs(from);
        for (int ix = from; ix <= to; ++ix) {
          Scalar const x = ix * side_length;
          Scalar const y = iy * side_length;
          Scalar z = x - y + 1.0;
          if (ix > 0) {
            z = -x - y + 1.0;
          }
          insert(x, z, y);
        }
        for (int ix = from + 1; ix <= to - 1; ++ix) {
          Scalar const x = ix * side_length;
          Scalar const y = iy * side_length;
          Scalar z = -x + y - 1.0;
          if (ix > 0) {
            z = x + y - 1.0;
          }
          insert(x, z, y);
        }
      }
      return samples;
    }

    Scalar cross(Vector2 const& origin, Vector2 const& a, Vector2 const& b) {
      Vector2 const oa = a - origin;
      Vector2 const ob = b - origin;
      return oa.x() * ob.y() - oa.y() * ob.x();
    }

    std::vector<int> convex_boundary(std::vector<Vector2> const& points) {
      if (points.size() <= 2) {
        std::vector<int> indices(points.size());
        std::iota(indices.begin(), indices.end(), 0);
        return indices;
      }
      std::vector<int> sorted(points.size());
      std::iota(sorted.begin(), sorted.end(), 0);
      std::sort(sorted.begin(), sorted.end(), [&](int a, int b) {
        if (
          points[static_cast<std::size_t>(a)].x() !=
          points[static_cast<std::size_t>(b)].x()) {
          return points[static_cast<std::size_t>(a)].x() <
            points[static_cast<std::size_t>(b)].x();
        }
        return points[static_cast<std::size_t>(a)].y() <
          points[static_cast<std::size_t>(b)].y();
      });

      std::vector<int> hull;
      for (int index : sorted) {
        while (hull.size() >= 2 &&
               cross(
                 points[static_cast<std::size_t>(hull[hull.size() - 2])],
                 points[static_cast<std::size_t>(hull.back())],
                 points[static_cast<std::size_t>(index)]) <= 0.0) {
          hull.pop_back();
        }
        hull.push_back(index);
      }
      std::size_t const lower_size = hull.size();
      for (auto iter = sorted.rbegin() + 1; iter != sorted.rend(); ++iter) {
        while (hull.size() > lower_size &&
               cross(
                 points[static_cast<std::size_t>(hull[hull.size() - 2])],
                 points[static_cast<std::size_t>(hull.back())],
                 points[static_cast<std::size_t>(*iter)]) <= 0.0) {
          hull.pop_back();
        }
        hull.push_back(*iter);
      }
      if (!hull.empty()) {
        hull.pop_back();
      }
      return hull;
    }

  }  // namespace

  stable_pose_result_t solve_stable_poses(
    stable_pose_problem_t const& problem, stable_pose_config_t const& config) {
    if (problem.body_model == nullptr) {
      return stable_pose_result_t {
        .status = solve_status_e::invalid_problem,
        .poses = {},
        .solver = {},
        .failure =
          {
            .code = "missing_body_model",
            .message = "stable-pose body model must not be null",
            .retryable = false,
          },
      };
    }
    if (
      !problem.com_offset_body.allFinite() || config.sampling_level < 0 ||
      config.sampling_level > 8 ||
      !std::isfinite(config.stable_eigenvalue_min)) {
      return stable_pose_result_t {
        .status = solve_status_e::invalid_problem,
        .poses = {},
        .solver = {},
        .failure =
          {
            .code = "invalid_config",
            .message =
              "stable-pose inputs must be finite and sampling level valid",
            .retryable = false,
          },
      };
    }

    std::optional<envelope_t> envelope = make_envelope(*problem.body_model);
    if (!envelope.has_value()) {
      return stable_pose_result_t {
        .status = solve_status_e::invalid_problem,
        .poses = {},
        .solver = {},
        .failure =
          {
            .code = "missing_dsf_geometry",
            .message = "stable-pose body requires DSF-Vert geometry",
            .retryable = false,
          },
      };
    }
    Vector3 const com_body = envelope->centroid_body + problem.com_offset_body;
    std::vector<Vector3> const samples = sphere_samples(config.sampling_level);
    std::vector<Vector3> solutions;
    solutions.reserve(samples.size());
    Scalar max_grad = 0.0;
    int total_iters = 0;
    for (Vector3 const& sample : samples) {
      equilibrium_t const equilibrium =
        solve_equilibrium(sample, *envelope->geometry, com_body);
      solutions.push_back(equilibrium.dir);
      max_grad = std::max(max_grad, equilibrium.grad_norm);
      total_iters += equilibrium.iters;
    }

    auto [equilibria, initial_mapping] = distinct_dirs(solutions);
    std::vector<Scalar> eigenvalues(equilibria.size());
    for (std::size_t i = 0; i < equilibria.size(); ++i) {
      eigenvalues[i] =
        min_eigenvalue(equilibria[i], *envelope->geometry, com_body);
    }
    std::vector<int> order(equilibria.size());
    std::iota(order.begin(), order.end(), 0);
    std::sort(order.begin(), order.end(), [&](int a, int b) {
      return eigenvalues[static_cast<std::size_t>(a)] >
        eigenvalues[static_cast<std::size_t>(b)];
    });

    std::vector<int> inverse_order(order.size());
    for (std::size_t i = 0; i < order.size(); ++i) {
      inverse_order[static_cast<std::size_t>(order[i])] = static_cast<int>(i);
    }
    std::vector<int> mapping(samples.size());
    for (std::size_t i = 0; i < samples.size(); ++i) {
      mapping[i] = inverse_order[static_cast<std::size_t>(initial_mapping[i])];
    }

    std::size_t stable_count = 0;
    while (stable_count < order.size() &&
           eigenvalues[static_cast<std::size_t>(order[stable_count])] >
             config.stable_eigenvalue_min) {
      ++stable_count;
    }
    stable_pose_result_t result {
      .status = stable_count == 0 ? solve_status_e::infeasible
                                  : solve_status_e::success,
      .poses = {},
      .reference_point_body = com_body,
      .solver =
        {
          .iters = total_iters,
          .converged = max_grad <= 1e-4,
          .objective = stable_count == 0
            ? 0.0
            : eigenvalues[static_cast<std::size_t>(order.front())],
          .grad_norm = max_grad,
        },
      .failure = {},
    };
    result.poses.reserve(stable_count);
    for (std::size_t stable = 0; stable < stable_count; ++stable) {
      Vector3 const& dir = equilibria[static_cast<std::size_t>(order[stable])];
      diffable_support_t const support =
        support_about(*envelope->geometry, dir, com_body);

      std::vector<Vector3> basin_samples;
      for (std::size_t i = 0; i < samples.size(); ++i) {
        if (mapping[i] == static_cast<int>(stable)) {
          basin_samples.push_back(samples[i]);
        }
      }
      Matrix32 const V_t = tangent_basis(dir);
      Matrix3 const proj = Matrix3::Identity() - dir * dir.transpose();
      std::vector<Vector2> projected;
      projected.reserve(basin_samples.size());
      for (Vector3 const& sample : basin_samples) {
        projected.push_back(V_t.transpose() * proj * (sample - dir));
      }
      std::vector<int> const boundary = convex_boundary(projected);
      Scalar max_cos = -1.0;
      for (int index : boundary) {
        max_cos = std::max(
          max_cos, basin_samples[static_cast<std::size_t>(index)].dot(dir));
      }
      result.poses.push_back(stable_pose_t {
        .resting_dir_body = dir,
        .cone_angle =
          std::acos(std::clamp(max_cos, Scalar {-1.0}, Scalar {1.0})),
        .support_point_body = support.s + com_body,
      });
    }
    if (result.status == solve_status_e::infeasible) {
      result.failure = planner_failure_t {
        .code = "no_stable_pose",
        .message = "no support equilibrium passed the stability threshold",
        .retryable = false,
      };
    }
    return result;
  }

}  // namespace stacking_core
