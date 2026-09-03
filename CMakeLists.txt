cmake_minimum_required(VERSION 3.16)

project(
  stacking-core
  VERSION 0.1.0
  LANGUAGES CXX
)

option(
  STACKING_CORE_BUILD_PYTHON
  "Build the optional stacking_core Python bindings"
  OFF
)

include(CMakePackageConfigHelpers)
include(GNUInstallDirs)

find_package(Eigen3 3.4 REQUIRED NO_MODULE)
find_package(tinyxml2 REQUIRED CONFIG)
find_package(Threads REQUIRED)

set(stacking_core_src_dir "${CMAKE_CURRENT_SOURCE_DIR}/src")

add_subdirectory(src)

include(CTest)

if(STACKING_CORE_BUILD_PYTHON)
  add_subdirectory(python)
endif()

if(BUILD_TESTING)
  add_subdirectory(test)
endif()

set(stacking_core_cmake_install_dir
    "${CMAKE_INSTALL_LIBDIR}/cmake/stacking-core")

configure_package_config_file(
  cmake/stacking-core-config.cmake.in
  "${CMAKE_CURRENT_BINARY_DIR}/stacking-core-config.cmake"
  INSTALL_DESTINATION "${stacking_core_cmake_install_dir}"
)

write_basic_package_version_file(
  "${CMAKE_CURRENT_BINARY_DIR}/stacking-core-config-version.cmake"
  VERSION "${PROJECT_VERSION}"
  COMPATIBILITY SameMajorVersion
)

install(
  EXPORT stacking-core-targets
  FILE stacking-core-targets.cmake
  NAMESPACE stacking-core::
  DESTINATION "${stacking_core_cmake_install_dir}"
)

install(
  FILES
    "${CMAKE_CURRENT_BINARY_DIR}/stacking-core-config.cmake"
    "${CMAKE_CURRENT_BINARY_DIR}/stacking-core-config-version.cmake"
  DESTINATION "${stacking_core_cmake_install_dir}"
)
