add_library(stacking-core-posegen STATIC)
add_library(stacking-core::posegen ALIAS stacking-core-posegen)

target_sources(
  stacking-core-posegen
  PRIVATE
    force_graph.cpp
    cone_qp.cpp
    objective.cpp
    pose_generator.cpp
)

target_compile_features(stacking-core-posegen PUBLIC cxx_std_20)
target_link_libraries(
  stacking-core-posegen
  PUBLIC stacking-core-core
)
set_target_properties(
  stacking-core-posegen
  PROPERTIES
    EXPORT_NAME posegen
    CXX_EXTENSIONS OFF
    POSITION_INDEPENDENT_CODE ON
)

install(
  TARGETS stacking-core-posegen
  EXPORT stacking-core-targets
  RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}"
  LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}"
  ARCHIVE DESTINATION "${CMAKE_INSTALL_LIBDIR}"
)
