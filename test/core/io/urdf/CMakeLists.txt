add_executable(stacking-core-urdf-loader-test loader_test.cpp)
target_link_libraries(
  stacking-core-urdf-loader-test
  PRIVATE stacking-core::core
)
add_test(NAME stacking-core.urdf-loader COMMAND stacking-core-urdf-loader-test)

add_executable(stacking-core-urdf-scene-test scene_test.cpp)
target_link_libraries(
  stacking-core-urdf-scene-test
  PRIVATE stacking-core::core
)
add_test(NAME stacking-core.urdf-scene COMMAND stacking-core-urdf-scene-test)
