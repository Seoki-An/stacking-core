add_executable(stacking-core-geometry-test geometry_test.cpp)
target_link_libraries(
  stacking-core-geometry-test
  PRIVATE stacking-core::core
)
add_test(NAME stacking-core.geometry COMMAND stacking-core-geometry-test)

add_executable(stacking-core-dsf-vert-test dsf_vert_test.cpp)
target_link_libraries(
  stacking-core-dsf-vert-test
  PRIVATE stacking-core::core
)
add_test(NAME stacking-core.dsf-vert COMMAND stacking-core-dsf-vert-test)
