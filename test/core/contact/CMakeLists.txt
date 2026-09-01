add_executable(stacking-core-contact-test contact_test.cpp)
target_link_libraries(
  stacking-core-contact-test
  PRIVATE stacking-core::core
)
add_test(NAME stacking-core.contact COMMAND stacking-core-contact-test)
