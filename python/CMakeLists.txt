find_package(Python3 COMPONENTS Interpreter Development REQUIRED)
find_package(pybind11 CONFIG REQUIRED)

pybind11_add_module(stacking-core-python MODULE bindings.cpp)
target_compile_features(stacking-core-python PRIVATE cxx_std_20)
target_link_libraries(
  stacking-core-python
  PRIVATE
    stacking-core-simulation
    stacking-core-posegen
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

set(
  STACKING_CORE_PYTHON_INSTALL_DIR
  "${CMAKE_INSTALL_LIBDIR}/python${Python3_VERSION_MAJOR}.${Python3_VERSION_MINOR}/site-packages"
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
      "${Python3_EXECUTABLE}" "${PROJECT_BINARY_DIR}/python/test_bindings.py"
  )
endif()
