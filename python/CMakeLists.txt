find_package(Python COMPONENTS Interpreter Development REQUIRED)
execute_process(
  COMMAND "${Python_EXECUTABLE}" -m nanobind --cmake_dir
  OUTPUT_VARIABLE nanobind_cmake_dir
  OUTPUT_STRIP_TRAILING_WHITESPACE
  RESULT_VARIABLE nanobind_cmake_result
)
if(NOT nanobind_cmake_result EQUAL 0)
  message(FATAL_ERROR "Could not locate the nanobind CMake package")
endif()
list(APPEND CMAKE_PREFIX_PATH "${nanobind_cmake_dir}")
find_package(nanobind CONFIG REQUIRED)

nanobind_add_module(
  stacking-core-python
  bindings.cpp
  bindings_simulation.cpp
  bindings_scene.cpp
  bindings_kinematics.cpp
  bindings_planner_types.cpp
  bindings_planner_grasp.cpp
  bindings_planner_motion.cpp
  bindings_planner_pick_place.cpp
)
target_compile_features(stacking-core-python PRIVATE cxx_std_20)
target_link_libraries(
  stacking-core-python
  PRIVATE
    stacking-core-simulation
    stacking-core-posegen
    stacking-core-planner
)
set(
  STACKING_CORE_PYTHON_OUTPUT_DIR
  "${PROJECT_BINARY_DIR}/python/stacking_core"
  CACHE PATH
  "Build output directory for the stacking_core Python extension"
)
set_target_properties(
  stacking-core-python
  PROPERTIES
    CXX_EXTENSIONS OFF
    OUTPUT_NAME _native
    LIBRARY_OUTPUT_DIRECTORY "${STACKING_CORE_PYTHON_OUTPUT_DIR}"
)

configure_file(
  stacking_core/__init__.py
  "${STACKING_CORE_PYTHON_OUTPUT_DIR}/__init__.py"
  COPYONLY
)
configure_file(
  test_bindings.py
  "${PROJECT_BINARY_DIR}/python/test_bindings.py"
  COPYONLY
)
file(
  COPY "${PROJECT_SOURCE_DIR}/test/planner/assets"
  DESTINATION "${PROJECT_BINARY_DIR}/python"
)

set(
  STACKING_CORE_PYTHON_INSTALL_DIR
  "${CMAKE_INSTALL_LIBDIR}/python${Python_VERSION_MAJOR}.${Python_VERSION_MINOR}/site-packages"
  CACHE PATH
  "Installation directory for the stacking_core Python package"
)

install(
  TARGETS stacking-core-python
  LIBRARY DESTINATION "${STACKING_CORE_PYTHON_INSTALL_DIR}/stacking_core"
)
install(
  FILES stacking_core/__init__.py
  DESTINATION "${STACKING_CORE_PYTHON_INSTALL_DIR}/stacking_core"
)

if(BUILD_TESTING)
  add_test(
    NAME stacking-core-python-test
    COMMAND
      "${CMAKE_COMMAND}" -E env
      "PYTHONPATH=${PROJECT_BINARY_DIR}/python"
      "${Python_EXECUTABLE}" "${PROJECT_BINARY_DIR}/python/test_bindings.py"
  )
endif()
