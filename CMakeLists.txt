cmake_minimum_required(VERSION 3.8)
project(riptide_description_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

find_package(ignition-gazebo6 REQUIRED)
set(IGN_GAZEBO_VER ${ignition-gazebo6_VERSION_MAJOR})
message(STATUS "Compiling against Ignition Fortress")

find_package(ignition-plugin1 REQUIRED)
set(IGN_PLUGIN_VER ${ignition-plugin1_VERSION_MAJOR})
find_package(ignition-common4 REQUIRED)
set(IGN_COMMON_VER ${ignition-common4_VERSION_MAJOR})

include_directories(include)

# Add sources for each plugin to be registered.
add_library(riptide_plugins SHARED src/Thruster.cc)
set_property(TARGET riptide_plugins PROPERTY CXX_STANDARD 17)
target_link_libraries(riptide_plugins
  ignition-gazebo${IGN_GAZEBO_VER}::ignition-gazebo${IGN_GAZEBO_VER}
  ignition-common${IGN_COMMON_VER}::ignition-common${IGN_COMMON_VER}
  ignition-plugin${IGN_PLUGIN_VER}::ignition-plugin${IGN_PLUGIN_VER}
)

install(TARGETS riptide_plugins
  DESTINATION lib
)

install(DIRECTORY
  include/
  DESTINATION include
)

install(
  DIRECTORY config launch meshes urdf worlds
  DESTINATION share/${PROJECT_NAME}
)

install(
  FILES model.sdf model.config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(riptide_plugins ign_hardware_plugins)

ament_package()
