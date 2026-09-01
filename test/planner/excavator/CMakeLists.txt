add_executable(
  stacking-core-planner-excavator-closed-form-test
  closed_form_initializer_test.cpp
)
target_link_libraries(
  stacking-core-planner-excavator-closed-form-test
  PRIVATE stacking-core::planner-excavator
)
add_test(
  NAME stacking-core.planner-excavator-closed-form
  COMMAND stacking-core-planner-excavator-closed-form-test
)
