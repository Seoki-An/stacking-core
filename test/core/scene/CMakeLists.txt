add_executable(stacking-core-scene-snapshot-test snapshot_test.cpp)
target_link_libraries(
  stacking-core-scene-snapshot-test
  PRIVATE stacking-core::core
)
add_test(NAME stacking-core.scene-snapshot COMMAND stacking-core-scene-snapshot-test)

add_executable(stacking-core-scene-view-test view_test.cpp)
target_link_libraries(
  stacking-core-scene-view-test
  PRIVATE stacking-core::core
)
add_test(NAME stacking-core.scene-view COMMAND stacking-core-scene-view-test)
