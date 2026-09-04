add_executable(
  stacking-core-simulation-regression-test
  simulation_regression_test.cpp
)
target_link_libraries(
  stacking-core-simulation-regression-test
  PRIVATE stacking-core::simulation
)
add_test(
  NAME stacking-core.simulation-regression
  COMMAND stacking-core-simulation-regression-test
)

add_executable(stacking-core-simulation-test simulation_test.cpp)
target_link_libraries(
  stacking-core-simulation-test
  PRIVATE stacking-core::simulation
)
add_test(NAME stacking-core.simulation COMMAND stacking-core-simulation-test)

add_executable(stacking-core-simulation-stability-test stability_test.cpp)
target_link_libraries(
  stacking-core-simulation-stability-test
  PRIVATE stacking-core::simulation
)
add_test(
  NAME stacking-core.simulation-stability
  COMMAND stacking-core-simulation-stability-test
)
