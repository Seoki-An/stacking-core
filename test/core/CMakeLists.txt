add_executable(stacking-core-types-test types_test.cpp)
target_link_libraries(
  stacking-core-types-test
  PRIVATE stacking-core::core
)
add_test(NAME stacking-core.types COMMAND stacking-core-types-test)

add_executable(stacking-core-transform-test transform_test.cpp)
target_link_libraries(
  stacking-core-transform-test
  PRIVATE stacking-core::core
)
add_test(NAME stacking-core.transform COMMAND stacking-core-transform-test)

add_subdirectory(geometry)
add_subdirectory(body)
add_subdirectory(scene)
add_subdirectory(kinematics)
add_subdirectory(io)
add_subdirectory(collision)
add_subdirectory(contact)
