add_executable(stacking-core-collision-test collision_test.cpp)
target_link_libraries(
  stacking-core-collision-test
  PRIVATE stacking-core::core
)
add_test(NAME stacking-core.collision COMMAND stacking-core-collision-test)
