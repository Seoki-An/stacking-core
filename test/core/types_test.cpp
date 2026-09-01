#include <stacking_core/types.hpp>

#include <stdexcept>
#include <type_traits>
#include <unordered_map>

namespace {

void require(bool condition) {
  if (!condition) {
    throw std::runtime_error("types test requirement failed");
  }
}

}  // namespace

int main() {
  using namespace stacking_core;

  static_assert(!std::is_same_v<EntityId, BodyModelId>);
  static_assert(!std::is_same_v<EntityId, GeometryId>);
  static_assert(!std::is_same_v<EntityId, FrameId>);

  EntityId const invalid;
  EntityId const first {0};
  EntityId const second {1};

  require(!invalid.valid());
  require(!static_cast<bool>(invalid));
  require(first.valid());
  require(first.value() == 0);
  require(first < second);

  std::unordered_map<EntityId, int> values;
  values.emplace(first, 42);
  require(values.at(first) == 42);
}
