add_executable(stacking-core-body-model-test model_test.cpp)
target_link_libraries(
  stacking-core-body-model-test
  PRIVATE stacking-core::core
)
add_test(NAME stacking-core.body-model COMMAND stacking-core-body-model-test)

add_executable(stacking-core-body-instance-test instance_test.cpp)
target_link_libraries(
  stacking-core-body-instance-test
  PRIVATE stacking-core::core
)
add_test(NAME stacking-core.body-instance COMMAND stacking-core-body-instance-test)

add_executable(
  stacking-core-convex-support-envelope-test
  convex_support_envelope_test.cpp
)
target_link_libraries(
  stacking-core-convex-support-envelope-test
  PRIVATE stacking-core::core
)
add_test(
  NAME stacking-core.convex-support-envelope
  COMMAND stacking-core-convex-support-envelope-test
)
