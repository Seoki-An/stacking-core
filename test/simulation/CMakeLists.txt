add_executable(
  stacking-core-simulation-diffsim-reference-test
  diffsim_reference_test.cpp
)
target_link_libraries(
  stacking-core-simulation-diffsim-reference-test
  PRIVATE stacking-core::simulation
)
add_test(
  NAME stacking-core.simulation-diffsim-reference
  COMMAND stacking-core-simulation-diffsim-reference-test
)

add_executable(stacking-core-simulation-test simulation_test.cpp)
target_link_libraries(
  stacking-core-simulation-test
  PRIVATE stacking-core::simulation
)
add_test(NAME stacking-core.simulation COMMAND stacking-core-simulation-test)
