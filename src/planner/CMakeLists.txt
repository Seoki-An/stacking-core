add_library(stacking-core-planner STATIC)
add_library(stacking-core::planner ALIAS stacking-core-planner)

target_sources(
  stacking-core-planner
  PRIVATE
    inverse_kinematics.cpp
    manipulation.cpp
    motion.cpp
    pick_place.cpp
    regrasp.cpp
    result.cpp
    stable_pose.cpp
)

add_subdirectory(grasp)

target_compile_features(stacking-core-planner PUBLIC cxx_std_20)
target_link_libraries(
  stacking-core-planner
  PUBLIC stacking-core-core
)
set_target_properties(
  stacking-core-planner
  PROPERTIES
    EXPORT_NAME planner
    CXX_EXTENSIONS OFF
    POSITION_INDEPENDENT_CODE ON
)

install(
  TARGETS stacking-core-planner
  EXPORT stacking-core-targets
  RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}"
  LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}"
  ARCHIVE DESTINATION "${CMAKE_INSTALL_LIBDIR}"
)

add_subdirectory(excavator)
