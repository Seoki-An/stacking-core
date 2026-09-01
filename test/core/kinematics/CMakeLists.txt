add_executable(stacking-core-kinematic-model-test model_test.cpp)
target_link_libraries(
  stacking-core-kinematic-model-test
  PRIVATE stacking-core::core
)
add_test(
  NAME stacking-core.kinematic-model
  COMMAND stacking-core-kinematic-model-test
)

add_executable(stacking-core-forward-kinematics-test forward_kinematics_test.cpp)
target_link_libraries(
  stacking-core-forward-kinematics-test
  PRIVATE stacking-core::core
)
add_test(
  NAME stacking-core.forward-kinematics
  COMMAND stacking-core-forward-kinematics-test
)
