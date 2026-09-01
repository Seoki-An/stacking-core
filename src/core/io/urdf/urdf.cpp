#include <stacking_core/io/urdf.hpp>

#include "wavefront.hpp"

#include <tinyxml2.h>

#include <Eigen/Geometry>

#include <cmath>
#include <optional>
#include <sstream>
#include <string_view>
#include <unordered_map>
#include <unordered_set>
#include <utility>

namespace stacking_core {
namespace {

using XmlElement = tinyxml2::XMLElement;

[[noreturn]] void fail(XmlElement const& element, std::string const& message) {
  throw UrdfError(
    message + " at <" + element.Name() + "> line " +
    std::to_string(element.GetLineNum()));
}

std::string attribute(
  XmlElement const& element, char const* name, bool required = true) {
  char const* value = element.Attribute(name);
  if (value == nullptr) {
    if (required) {
      fail(element, "missing required attribute '" + std::string {name} + "'");
    }
    return {};
  }
  return value;
}

XmlElement const* child(
  XmlElement const& element, char const* name, bool required = true) {
  XmlElement const* result = element.FirstChildElement(name);
  if (result == nullptr) {
    if (required) {
      fail(element, "missing required child <" + std::string {name} + ">");
    }
    return nullptr;
  }
  if (result->NextSiblingElement(name) != nullptr) {
    fail(element, "repeated child <" + std::string {name} + ">");
  }
  return result;
}

Scalar parse_scalar(
  XmlElement const& element, char const* name, bool required = true) {
  std::string const text = attribute(element, name, required);
  if (text.empty() && !required) {
    return 0.0;
  }
  std::istringstream input {text};
  Scalar value = 0.0;
  std::string extra;
  if (!(input >> value) || !std::isfinite(value) || (input >> extra)) {
    fail(element, "attribute '" + std::string {name} + "' must be finite");
  }
  return value;
}

Vector3 parse_vector3(
  XmlElement const& element,
  char const* name,
  Vector3 default_value = Vector3::Zero()) {
  char const* text = element.Attribute(name);
  if (text == nullptr) {
    return default_value;
  }
  std::istringstream input {text};
  Vector3 value;
  std::string extra;
  if (!(input >> value.x() >> value.y() >> value.z()) ||
      !value.allFinite() || (input >> extra)) {
    fail(
      element,
      "attribute '" + std::string {name} +
        "' must contain exactly three finite values");
  }
  return value;
}

pose_t parse_origin(XmlElement const& parent) {
  XmlElement const* origin = child(parent, "origin", false);
  if (origin == nullptr) {
    return pose_t {};
  }
  Vector3 const position = parse_vector3(*origin, "xyz");
  Vector3 const rpy = parse_vector3(*origin, "rpy");
  Quaternion const orientation =
    Eigen::AngleAxis<Scalar> {rpy.z(), Vector3::UnitZ()} *
    Eigen::AngleAxis<Scalar> {rpy.y(), Vector3::UnitY()} *
    Eigen::AngleAxis<Scalar> {rpy.x(), Vector3::UnitX()};
  return pose_t {position, orientation};
}

std::optional<inertial_t> parse_inertial(XmlElement const& link) {
  XmlElement const* inertial = child(link, "inertial", false);
  if (inertial == nullptr) {
    return std::nullopt;
  }
  XmlElement const* mass = child(*inertial, "mass");
  XmlElement const* inertia = child(*inertial, "inertia");
  Scalar const ixx = parse_scalar(*inertia, "ixx");
  Scalar const ixy = parse_scalar(*inertia, "ixy");
  Scalar const ixz = parse_scalar(*inertia, "ixz");
  Scalar const iyy = parse_scalar(*inertia, "iyy");
  Scalar const iyz = parse_scalar(*inertia, "iyz");
  Scalar const izz = parse_scalar(*inertia, "izz");
  Matrix3 inertia_matrix;
  inertia_matrix <<
    ixx, ixy, ixz,
    ixy, iyy, iyz,
    ixz, iyz, izz;
  return inertial_t {
    .body_from_inertial = parse_origin(*inertial),
    .mass = parse_scalar(*mass, "value"),
    .inertia = inertia_matrix,
  };
}

std::filesystem::path resolve_resource(
  std::filesystem::path const& urdf_directory,
  XmlElement const& element,
  std::string const& filename) {
  if (filename.find("://") != std::string::npos) {
    fail(
      element,
      "resource URI schemes are not supported yet: '" + filename + "'");
  }
  std::filesystem::path resource {filename};
  if (resource.is_relative()) {
    std::filesystem::path directory = urdf_directory;
    while (true) {
      std::filesystem::path const candidate =
        (directory / resource).lexically_normal();
      if (std::filesystem::exists(candidate)) {
        return candidate;
      }
      std::filesystem::path const parent = directory.parent_path();
      if (parent == directory || parent.empty()) {
        break;
      }
      directory = parent;
    }
    resource = urdf_directory / resource;
  }
  return resource.lexically_normal();
}

std::vector<geometry_config_t> parse_collision_geometry(
  XmlElement const& link,
  std::filesystem::path const& urdf_directory,
  std::uint64_t& next_geometry_id) {
  std::vector<geometry_config_t> geometries;
  for (XmlElement const* collision = link.FirstChildElement("collision");
       collision != nullptr;
       collision = collision->NextSiblingElement("collision")) {
    XmlElement const* geometry = child(*collision, "geometry");
    XmlElement const* dsf = child(*geometry, "dsf_vert", false);
    if (dsf == nullptr) {
      continue;
    }
    std::filesystem::path const resource = resolve_resource(
      urdf_directory, *dsf, attribute(*dsf, "filename"));
    std::vector<detail::dsf_mesh_t> meshes;
    try {
      meshes = detail::load_dsf_meshes(resource);
    } catch (std::exception const& error) {
      fail(*dsf, error.what());
    }

    pose_t const body_from_geometry = parse_origin(*collision);
    for (detail::dsf_mesh_t& mesh : meshes) {
      if (dsf->Attribute("sharpness") != nullptr) {
        Scalar const sharpness = parse_scalar(*dsf, "sharpness");
        if (sharpness < 2.0 || std::floor(sharpness) != sharpness) {
          fail(*dsf, "sharpness must be an integer >= 2");
        }
        mesh.sharpness = static_cast<int>(sharpness);
      }
      if (dsf->Attribute("mu") != nullptr) {
        mesh.friction = parse_scalar(*dsf, "mu");
      }
      geometries.push_back(dsf_vert_geometry_config_t {
        .properties = geometry_properties_t {
          .id = GeometryId {next_geometry_id++},
          .body_from_geometry = body_from_geometry,
          .material = material_t {.friction = mesh.friction},
        },
        .nodes = std::move(mesh.nodes),
        .sharpness = mesh.sharpness,
      });
    }
  }
  return geometries;
}

enum class parsed_joint_type_e {
  fixed,
  revolute,
  prismatic,
  floating,
};

parsed_joint_type_e parse_joint_type(XmlElement const& joint) {
  std::string const type = attribute(joint, "type");
  if (type == "fixed") {
    return parsed_joint_type_e::fixed;
  }
  if (type == "revolute" || type == "continuous") {
    return parsed_joint_type_e::revolute;
  }
  if (type == "prismatic") {
    return parsed_joint_type_e::prismatic;
  }
  if (type == "floating") {
    return parsed_joint_type_e::floating;
  }
  fail(joint, "unsupported URDF joint type '" + type + "'");
}

joint_type_e to_joint_type(parsed_joint_type_e type) {
  switch (type) {
  case parsed_joint_type_e::fixed:
    return joint_type_e::fixed;
  case parsed_joint_type_e::revolute:
    return joint_type_e::revolute;
  case parsed_joint_type_e::prismatic:
    return joint_type_e::prismatic;
  case parsed_joint_type_e::floating:
    break;
  }
  throw std::logic_error("floating URDF joint has no scalar joint type");
}

struct parsed_joint_t {
  XmlElement const* element = nullptr;
  JointId id;
  std::string name;
  parsed_joint_type_e type = parsed_joint_type_e::fixed;
  LinkId parent;
  LinkId child;
  pose_t parent_from_child_zero;
  Vector3 axis = Vector3::UnitX();
  std::optional<joint_limit_t> limit;
  std::optional<std::string> mimic_source;
  Scalar mimic_multiplier = 1.0;
  Scalar mimic_offset = 0.0;
};

std::optional<joint_limit_t> parse_limit(
  XmlElement const& joint, parsed_joint_type_e type) {
  if (type != parsed_joint_type_e::revolute &&
      type != parsed_joint_type_e::prismatic) {
    return std::nullopt;
  }
  if (attribute(joint, "type") == "continuous") {
    return std::nullopt;
  }
  XmlElement const* limit = child(joint, "limit", false);
  if (limit == nullptr) {
    return std::nullopt;
  }
  bool const has_lower = limit->Attribute("lower") != nullptr;
  bool const has_upper = limit->Attribute("upper") != nullptr;
  if (has_lower != has_upper) {
    fail(*limit, "joint limit must provide both lower and upper bounds");
  }
  if (!has_lower) {
    return std::nullopt;
  }
  return joint_limit_t {
    .lower = parse_scalar(*limit, "lower"),
    .upper = parse_scalar(*limit, "upper"),
  };
}

}  // namespace

UrdfModel::UrdfModel(
  std::string name,
  std::shared_ptr<KinematicModel const> kinematics,
  std::vector<urdf_link_model_t> links)
    : name_(std::move(name)),
      kinematics_(std::move(kinematics)),
      links_(std::move(links)) {
  if (name_.empty()) {
    throw std::invalid_argument("URDF model name must not be empty");
  }
  if (kinematics_ == nullptr) {
    throw std::invalid_argument("URDF kinematic model must not be null");
  }
  if (links_.size() != kinematics_->linkCount()) {
    throw std::invalid_argument("URDF body model count must match link count");
  }
  std::unordered_set<LinkId> link_ids;
  for (urdf_link_model_t const& link : links_) {
    if (link.body_model == nullptr || kinematics_->findLink(link.link) == nullptr) {
      throw std::invalid_argument("URDF link binding must reference valid models");
    }
    if (!link_ids.emplace(link.link).second) {
      throw std::invalid_argument("URDF link bindings must be unique");
    }
  }
}

BodyModel const& UrdfModel::bodyModel(LinkId link) const {
  return *bodyModelPtr(link);
}

std::shared_ptr<BodyModel const> const& UrdfModel::bodyModelPtr(
  LinkId link) const {
  for (urdf_link_model_t const& binding : links_) {
    if (binding.link == link) {
      return binding.body_model;
    }
  }
  throw std::out_of_range("URDF model does not contain the link ID");
}

UrdfModel load_urdf_model(std::filesystem::path const& path) {
  std::filesystem::path const absolute_path =
    std::filesystem::absolute(path).lexically_normal();
  tinyxml2::XMLDocument document;
  tinyxml2::XMLError const result = document.LoadFile(absolute_path.string().c_str());
  if (result != tinyxml2::XML_SUCCESS) {
    throw UrdfError(
      "failed to load URDF '" + absolute_path.string() + "': " +
      document.ErrorStr());
  }
  XmlElement const* robot = document.RootElement();
  if (robot == nullptr || std::string_view {robot->Name()} != "robot") {
    throw UrdfError("URDF root element must be <robot>");
  }
  std::string const robot_name = attribute(*robot, "name");

  std::vector<kinematic_link_t> kinematic_links;
  std::vector<XmlElement const*> link_elements;
  std::unordered_map<std::string, LinkId> link_ids;
  std::uint64_t next_link_id = 0;
  for (XmlElement const* link = robot->FirstChildElement("link");
       link != nullptr;
       link = link->NextSiblingElement("link")) {
    std::string const name = attribute(*link, "name");
    LinkId const id {next_link_id++};
    if (!link_ids.emplace(name, id).second) {
      fail(*link, "duplicate URDF link name '" + name + "'");
    }
    kinematic_links.push_back(kinematic_link_t {.id = id, .name = name});
    link_elements.push_back(link);
  }
  if (kinematic_links.empty()) {
    fail(*robot, "URDF requires at least one link");
  }

  std::vector<parsed_joint_t> parsed_joints;
  std::unordered_map<std::string, JointId> included_joint_ids;
  std::unordered_map<LinkId, std::size_t> incoming_joints;
  std::uint64_t next_joint_id = 0;
  for (XmlElement const* joint = robot->FirstChildElement("joint");
       joint != nullptr;
       joint = joint->NextSiblingElement("joint")) {
    parsed_joint_t parsed;
    parsed.element = joint;
    parsed.id = JointId {next_joint_id++};
    parsed.name = attribute(*joint, "name");
    parsed.type = parse_joint_type(*joint);

    std::string const parent_name = attribute(*child(*joint, "parent"), "link");
    std::string const child_name = attribute(*child(*joint, "child"), "link");
    auto const parent = link_ids.find(parent_name);
    auto const child_link = link_ids.find(child_name);
    if (parent == link_ids.end() || child_link == link_ids.end()) {
      fail(*joint, "joint parent and child must name declared links");
    }
    parsed.parent = parent->second;
    parsed.child = child_link->second;
    if (!incoming_joints.emplace(parsed.child, parsed_joints.size()).second) {
      fail(*joint, "URDF link cannot have multiple parent joints");
    }

    parsed.parent_from_child_zero = parse_origin(*joint);
    if (parsed.type == parsed_joint_type_e::revolute ||
        parsed.type == parsed_joint_type_e::prismatic) {
      XmlElement const* axis = child(*joint, "axis", false);
      parsed.axis = axis == nullptr
        ? Vector3::UnitX()
        : parse_vector3(*axis, "xyz", Vector3::UnitX());
    }
    parsed.limit = parse_limit(*joint, parsed.type);

    XmlElement const* mimic = child(*joint, "mimic", false);
    if (mimic != nullptr) {
      parsed.mimic_source = attribute(*mimic, "joint");
      if (mimic->Attribute("multiplier") != nullptr) {
        parsed.mimic_multiplier = parse_scalar(*mimic, "multiplier");
      }
      if (mimic->Attribute("offset") != nullptr) {
        parsed.mimic_offset = parse_scalar(*mimic, "offset");
      }
    }
    if (parsed.type != parsed_joint_type_e::floating &&
        !included_joint_ids.emplace(parsed.name, parsed.id).second) {
      fail(*joint, "duplicate URDF joint name '" + parsed.name + "'");
    }
    if (parsed.type == parsed_joint_type_e::floating) {
      for (parsed_joint_t const& previous : parsed_joints) {
        if (previous.name == parsed.name) {
          fail(*joint, "duplicate URDF joint name '" + parsed.name + "'");
        }
      }
    }
    parsed_joints.push_back(std::move(parsed));
  }

  std::unordered_set<std::string> all_joint_names;
  for (parsed_joint_t const& parsed : parsed_joints) {
    if (!all_joint_names.emplace(parsed.name).second) {
      fail(*parsed.element, "duplicate URDF joint name '" + parsed.name + "'");
    }
    if (parsed.type == parsed_joint_type_e::floating &&
        incoming_joints.contains(parsed.parent)) {
      fail(
        *parsed.element,
        "floating joints are supported only from a root parent link");
    }
  }

  std::vector<kinematic_joint_t> kinematic_joints;
  for (parsed_joint_t const& parsed : parsed_joints) {
    if (parsed.type == parsed_joint_type_e::floating) {
      // A root floating joint is represented by making its child another root;
      // KinematicState then stores its frame-explicit root pose directly.
      continue;
    }
    std::optional<joint_mimic_t> mimic;
    if (parsed.mimic_source.has_value()) {
      auto const source = included_joint_ids.find(*parsed.mimic_source);
      if (source == included_joint_ids.end()) {
        fail(*parsed.element, "mimic source must be a scalar URDF joint");
      }
      mimic = joint_mimic_t {
        .source = source->second,
        .multiplier = parsed.mimic_multiplier,
        .offset = parsed.mimic_offset,
      };
    }
    kinematic_joints.push_back(kinematic_joint_t {
      .id = parsed.id,
      .name = parsed.name,
      .type = to_joint_type(parsed.type),
      .parent = parsed.parent,
      .child = parsed.child,
      .parent_from_child_zero = parsed.parent_from_child_zero,
      .axis = parsed.axis,
      .limit = parsed.limit,
      .mimic = mimic,
    });
  }

  std::shared_ptr<KinematicModel const> kinematics;
  try {
    kinematics = std::make_shared<KinematicModel>(kinematic_model_config_t {
      .links = std::move(kinematic_links),
      .joints = std::move(kinematic_joints),
    });
  } catch (std::exception const& error) {
    throw UrdfError("invalid URDF kinematic model: " + std::string {error.what()});
  }

  std::vector<urdf_link_model_t> link_models;
  link_models.reserve(link_elements.size());
  std::uint64_t next_geometry_id = 0;
  for (std::size_t index = 0; index < link_elements.size(); ++index) {
    LinkId const link_id {static_cast<LinkId::value_type>(index)};
    body_model_config_t body_config {
      .id = BodyModelId {static_cast<BodyModelId::value_type>(index)},
      .inertial = parse_inertial(*link_elements[index]),
      .geometries = parse_collision_geometry(
        *link_elements[index], absolute_path.parent_path(), next_geometry_id),
    };
    try {
      link_models.push_back(urdf_link_model_t {
        .link = link_id,
        .body_model = std::make_shared<BodyModel>(std::move(body_config)),
      });
    } catch (std::exception const& error) {
      fail(
        *link_elements[index],
        "invalid body model for link '" +
          kinematics->link(link_id).name + "': " + error.what());
    }
  }

  return UrdfModel {robot_name, std::move(kinematics), std::move(link_models)};
}

}  // namespace stacking_core
