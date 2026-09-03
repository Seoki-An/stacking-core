add_executable(stacking-core-planner-types-test types_test.cpp)
target_link_libraries(
  stacking-core-planner-types-test
  PRIVATE stacking-core::planner
)
add_test(NAME stacking-core.planner-types COMMAND stacking-core-planner-types-test)

add_executable(
  stacking-core-planner-diffsim-reference-test
  diffsim_reference_test.cpp
)
target_link_libraries(
  stacking-core-planner-diffsim-reference-test
  PRIVATE stacking-core::planner
)
add_test(
  NAME stacking-core.planner-diffsim-reference
  COMMAND stacking-core-planner-diffsim-reference-test
)

add_executable(
  stacking-core-planner-diffsim-motion-reference-test
  diffsim_motion_reference_test.cpp
)
target_link_libraries(
  stacking-core-planner-diffsim-motion-reference-test
  PRIVATE stacking-core::planner
)
add_test(
  NAME stacking-core.planner-diffsim-motion-reference
  COMMAND stacking-core-planner-diffsim-motion-reference-test
)

add_executable(
  stacking-core-planner-stacking-planner-state-parity-test
  stacking_planner_state_parity_test.cpp
)
target_link_libraries(
  stacking-core-planner-stacking-planner-state-parity-test
  PRIVATE stacking-core::planner
)
add_test(
  NAME stacking-core.planner-stacking-planner-state-parity
  COMMAND stacking-core-planner-stacking-planner-state-parity-test
)

add_executable(
  stacking-core-planner-inverse-kinematics-test
  inverse_kinematics_test.cpp
)
target_link_libraries(
  stacking-core-planner-inverse-kinematics-test
  PRIVATE stacking-core::planner
)
add_test(
  NAME stacking-core.planner-inverse-kinematics
  COMMAND stacking-core-planner-inverse-kinematics-test
)

add_executable(stacking-core-planner-motion-test motion_test.cpp)
target_link_libraries(
  stacking-core-planner-motion-test
  PRIVATE stacking-core::planner
)
add_test(NAME stacking-core.planner-motion COMMAND stacking-core-planner-motion-test)

add_executable(stacking-core-planner-parallel-test parallel_test.cpp)
target_include_directories(
  stacking-core-planner-parallel-test
  PRIVATE "${PROJECT_SOURCE_DIR}/src/planner"
)
target_link_libraries(
  stacking-core-planner-parallel-test
  PRIVATE stacking-core::planner
)
add_test(
  NAME stacking-core.planner-parallel
  COMMAND stacking-core-planner-parallel-test
)

add_executable(stacking-core-planner-manipulation-test manipulation_test.cpp)
target_link_libraries(
  stacking-core-planner-manipulation-test
  PRIVATE stacking-core::planner
)
add_test(
  NAME stacking-core.planner-manipulation
  COMMAND stacking-core-planner-manipulation-test
)

add_executable(stacking-core-planner-regrasp-test regrasp_test.cpp)
target_link_libraries(
  stacking-core-planner-regrasp-test
  PRIVATE stacking-core::planner
)
add_test(NAME stacking-core.planner-regrasp COMMAND stacking-core-planner-regrasp-test)

add_executable(stacking-core-planner-grasp-test grasp_test.cpp)
target_include_directories(
  stacking-core-planner-grasp-test
  PRIVATE "${PROJECT_SOURCE_DIR}/src/planner/grasp"
)
target_link_libraries(
  stacking-core-planner-grasp-test
  PRIVATE stacking-core::planner
)
add_test(NAME stacking-core.planner-grasp COMMAND stacking-core-planner-grasp-test)

add_executable(
  stacking-core-planner-grasp-sampling-test
  grasp_sampling_test.cpp
)
target_link_libraries(
  stacking-core-planner-grasp-sampling-test
  PRIVATE stacking-core::planner
)
add_test(
  NAME stacking-core.planner-grasp-sampling
  COMMAND stacking-core-planner-grasp-sampling-test
)

add_subdirectory(excavator)
