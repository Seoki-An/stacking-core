add_subdirectory(core)
add_subdirectory(simulation)
add_subdirectory(posegen)
add_subdirectory(planner)

# Convenience target for consumers that want every stacking-core module.
add_library(stacking-core INTERFACE)
add_library(stacking-core::stacking-core ALIAS stacking-core)
target_link_libraries(
  stacking-core
  INTERFACE
    stacking-core-simulation
    stacking-core-posegen
    stacking-core-planner
)
set_target_properties(stacking-core PROPERTIES EXPORT_NAME stacking-core)

install(TARGETS stacking-core EXPORT stacking-core-targets)
install(
  DIRECTORY "${PROJECT_SOURCE_DIR}/include/"
  DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}"
)
