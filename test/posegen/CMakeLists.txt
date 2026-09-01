add_executable(stacking-core-posegen-test posegen_test.cpp)
target_link_libraries(
  stacking-core-posegen-test
  PRIVATE stacking-core::posegen
)
add_test(NAME stacking-core.posegen COMMAND stacking-core-posegen-test)
