add_library(stacking-core-planner-excavator STATIC)
add_library(
  stacking-core::planner-excavator
  ALIAS stacking-core-planner-excavator
)

target_sources(
  stacking-core-planner-excavator
  PRIVATE closed_form_initializer.cpp
)
target_compile_features(stacking-core-planner-excavator PUBLIC cxx_std_20)
target_link_libraries(
  stacking-core-planner-excavator
  PUBLIC stacking-core-planner
)
set_target_properties(
  stacking-core-planner-excavator
  PROPERTIES
    EXPORT_NAME planner-excavator
    CXX_EXTENSIONS OFF
    POSITION_INDEPENDENT_CODE ON
)

install(
  TARGETS stacking-core-planner-excavator
  EXPORT stacking-core-targets
  RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}"
  LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}"
  ARCHIVE DESTINATION "${CMAKE_INSTALL_LIBDIR}"
)
