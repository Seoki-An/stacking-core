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

add_executable(stacking-core-constraint-solver-test constraint_solver_test.cpp)
target_include_directories(
  stacking-core-constraint-solver-test PRIVATE ${stacking_core_src_dir}/simulation
)
target_link_libraries(
  stacking-core-constraint-solver-test PRIVATE stacking-core::simulation
)
add_test(NAME stacking-core.constraint-solver COMMAND stacking-core-constraint-solver-test)

# Optional diagnostic: frozen contact problems with analytical optima. It is
# intentionally excluded from CTest and the default build (timings are noisy).
add_executable(
  stacking-core-constraint-solver-benchmark EXCLUDE_FROM_ALL
  constraint_solver_benchmark.cpp
)
target_include_directories(
  stacking-core-constraint-solver-benchmark PRIVATE ${stacking_core_src_dir}/simulation
)
target_link_libraries(
  stacking-core-constraint-solver-benchmark PRIVATE stacking-core::simulation
)

add_executable(stacking-core-simulation-stability-test stability_test.cpp)
target_link_libraries(
  stacking-core-simulation-stability-test
  PRIVATE stacking-core::simulation
)
add_test(
  NAME stacking-core.simulation-stability
  COMMAND stacking-core-simulation-stability-test
)
