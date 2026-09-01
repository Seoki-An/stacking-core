add_library(stacking-core-simulation STATIC)
add_library(stacking-core::simulation ALIAS stacking-core-simulation)

target_sources(
  stacking-core-simulation
  PRIVATE
    constraint_solver.cpp
    integrator.cpp
    simulator.cpp
)

target_compile_features(stacking-core-simulation PUBLIC cxx_std_20)
target_link_libraries(
  stacking-core-simulation
  PUBLIC stacking-core-core
)
set_target_properties(
  stacking-core-simulation
  PROPERTIES
    EXPORT_NAME simulation
    CXX_EXTENSIONS OFF
    POSITION_INDEPENDENT_CODE ON
)

install(
  TARGETS stacking-core-simulation
  EXPORT stacking-core-targets
  RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}"
  LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}"
  ARCHIVE DESTINATION "${CMAKE_INSTALL_LIBDIR}"
)
