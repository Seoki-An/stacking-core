add_library(stacking-core-core STATIC)
add_library(stacking-core::core ALIAS stacking-core-core)

target_sources(
  stacking-core-core
  PRIVATE
    transform.cpp
)

add_subdirectory(geometry)
add_subdirectory(body)
add_subdirectory(scene)
add_subdirectory(kinematics)
add_subdirectory(io)
add_subdirectory(collision)
add_subdirectory(contact)

target_compile_features(stacking-core-core PUBLIC cxx_std_20)
target_link_libraries(
  stacking-core-core
  PUBLIC Eigen3::Eigen
  PRIVATE tinyxml2::tinyxml2
)

target_include_directories(
  stacking-core-core
  PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

set_target_properties(
  stacking-core-core
  PROPERTIES
    EXPORT_NAME core
    CXX_EXTENSIONS OFF
    POSITION_INDEPENDENT_CODE ON
)

install(
  TARGETS stacking-core-core
  EXPORT stacking-core-targets
  RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}"
  LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}"
  ARCHIVE DESTINATION "${CMAKE_INSTALL_LIBDIR}"
)
