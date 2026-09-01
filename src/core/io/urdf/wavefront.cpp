#include "wavefront.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <fstream>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

namespace stacking_core::detail {
  namespace {

    struct shape_t {
      std::vector<std::size_t> declared_vertices;
      std::vector<std::size_t> referenced_vertices;
      int sharpness = 20;
      Scalar friction = 1.0;
    };

    std::string_view trim(std::string_view value) {
      while (!value.empty() &&
             std::isspace(static_cast<unsigned char>(value.front()))) {
        value.remove_prefix(1);
      }
      while (!value.empty() &&
             std::isspace(static_cast<unsigned char>(value.back()))) {
        value.remove_suffix(1);
      }
      return value;
    }

    [[noreturn]] void fail(
      std::filesystem::path const& path, std::size_t line,
      std::string const& message) {
      throw std::runtime_error(
        "failed to load DSF OBJ '" + path.string() + "' at line " +
        std::to_string(line) + ": " + message);
    }

    Scalar parse_scalar(
      std::filesystem::path const& path, std::size_t line,
      std::string_view value, std::string_view field) {
      std::istringstream input {std::string {trim(value)}};
      Scalar result = 0.0;
      std::string extra;
      if (!(input >> result) || !std::isfinite(result) || (input >> extra)) {
        fail(path, line, "invalid " + std::string {field});
      }
      return result;
    }

    void parse_metadata(
      std::filesystem::path const& path, std::size_t line_number,
      std::string_view comment, shape_t& shape) {
      comment = trim(comment);
      std::size_t const separator = comment.find(':');
      if (separator == std::string_view::npos) {
        return;
      }
      std::string_view const key = trim(comment.substr(0, separator));
      std::string_view const value = comment.substr(separator + 1);
      if (key == "sharpness") {
        Scalar const parsed =
          parse_scalar(path, line_number, value, "sharpness metadata");
        if (parsed < 2.0 || std::floor(parsed) != parsed) {
          fail(path, line_number, "sharpness metadata must be an integer >= 2");
        }
        shape.sharpness = static_cast<int>(parsed);
      } else if (key == "mu") {
        Scalar const parsed =
          parse_scalar(path, line_number, value, "mu metadata");
        if (parsed < 0.0) {
          fail(path, line_number, "mu metadata must be nonnegative");
        }
        shape.friction = parsed;
      }
    }

    std::size_t parse_vertex_index(
      std::filesystem::path const& path, std::size_t line_number,
      std::string const& token, std::size_t vertex_count) {
      std::string const index_text = token.substr(0, token.find('/'));
      std::istringstream input {index_text};
      long long index = 0;
      std::string extra;
      if (!(input >> index) || index == 0 || (input >> extra)) {
        fail(path, line_number, "invalid face vertex index");
      }
      long long const resolved =
        index > 0 ? index - 1 : static_cast<long long>(vertex_count) + index;
      if (resolved < 0 || resolved >= static_cast<long long>(vertex_count)) {
        fail(path, line_number, "face vertex index is out of range");
      }
      return static_cast<std::size_t>(resolved);
    }

  }  // namespace

  std::vector<dsf_mesh_t> load_dsf_meshes(std::filesystem::path const& path) {
    std::string extension = path.extension().string();
    std::transform(
      extension.begin(), extension.end(), extension.begin(),
      [](unsigned char value) {
        return static_cast<char>(std::tolower(value));
      });
    if (extension != ".obj") {
      throw std::runtime_error(
        "DSF geometry currently requires an OBJ file: '" + path.string() + "'");
    }

    std::ifstream input {path};
    if (!input) {
      throw std::runtime_error(
        "cannot open DSF OBJ file: '" + path.string() + "'");
    }

    std::vector<Vector3> vertices;
    std::vector<shape_t> shapes;
    shape_t global_properties;
    shape_t* current_shape = nullptr;
    std::string line;
    std::size_t line_number = 0;
    while (std::getline(input, line)) {
      ++line_number;
      std::string_view content = trim(line);
      if (content.empty()) {
        continue;
      }
      if (content.front() == '#') {
        shape_t& properties =
          current_shape == nullptr ? global_properties : *current_shape;
        parse_metadata(path, line_number, content.substr(1), properties);
        continue;
      }

      std::istringstream line_input {std::string {content}};
      std::string keyword;
      line_input >> keyword;
      if (keyword == "o" || keyword == "g") {
        shapes.push_back(global_properties);
        current_shape = &shapes.back();
      } else if (keyword == "v") {
        Vector3 vertex;
        std::string extra;
        if (
          !(line_input >> vertex.x() >> vertex.y() >> vertex.z()) ||
          !vertex.allFinite() || (line_input >> extra)) {
          fail(
            path, line_number,
            "vertex must contain exactly three finite values");
        }
        vertices.push_back(vertex);
        if (current_shape != nullptr) {
          current_shape->declared_vertices.push_back(vertices.size() - 1);
        }
      } else if (keyword == "f") {
        if (current_shape == nullptr) {
          shapes.push_back(global_properties);
          current_shape = &shapes.back();
        }
        std::string token;
        std::size_t face_size = 0;
        while (line_input >> token) {
          current_shape->referenced_vertices.push_back(
            parse_vertex_index(path, line_number, token, vertices.size()));
          ++face_size;
        }
        if (face_size < 3) {
          fail(
            path, line_number, "face must reference at least three vertices");
        }
      }
    }
    if (vertices.empty()) {
      throw std::runtime_error(
        "DSF OBJ contains no vertices: '" + path.string() + "'");
    }
    if (shapes.empty()) {
      shapes.push_back(global_properties);
    }

    std::vector<dsf_mesh_t> meshes;
    for (shape_t const& shape : shapes) {
      std::vector<std::size_t> selected;
      if (shapes.size() == 1) {
        selected.resize(vertices.size());
        for (std::size_t index = 0; index < vertices.size(); ++index) {
          selected[index] = index;
        }
      } else {
        std::set<std::size_t> const unique_indices {
          shape.referenced_vertices.begin(), shape.referenced_vertices.end()};
        selected.assign(unique_indices.begin(), unique_indices.end());
        if (selected.empty()) {
          selected = shape.declared_vertices;
        }
      }
      if (selected.empty()) {
        continue;
      }

      dsf_mesh_t mesh;
      mesh.nodes.resize(3, static_cast<Eigen::Index>(selected.size()));
      for (std::size_t index = 0; index < selected.size(); ++index) {
        mesh.nodes.col(static_cast<Eigen::Index>(index)) =
          vertices[selected[index]];
      }
      mesh.sharpness = shape.sharpness;
      mesh.friction = shape.friction;
      meshes.push_back(std::move(mesh));
    }
    if (meshes.empty()) {
      throw std::runtime_error(
        "DSF OBJ contains no usable shapes: '" + path.string() + "'");
    }
    return meshes;
  }

}  // namespace stacking_core::detail
