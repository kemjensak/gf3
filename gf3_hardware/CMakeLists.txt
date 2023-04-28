cmake_minimum_required(VERSION 3.8)
project(gf3_hardware)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  ament_cmake
  rclcpp
  can_msgs
  hardware_interface
  pluginlib
  rclcpp_lifecycle
  nav_msgs
  tf2
  tf2_ros
  moteus_msgs
)


foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

add_library(${PROJECT_NAME}
        SHARED
        src/gf3_hardware_interface.cpp
        src/CanBridge.cpp
        src/myactuator.cpp
        src/MoteusAPI.cpp)

target_include_directories(
  ${PROJECT_NAME}
  PRIVATE
  include
)

ament_target_dependencies( ${PROJECT_NAME}
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(${PROJECT_NAME} PRIVATE "JOINT_STATE_BROADCASTER_BUILDING_DLL")
# prevent pluginlib from using boost
target_compile_definitions(${PROJECT_NAME} PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")
pluginlib_export_plugin_description_file(hardware_interface gf3_hardware.xml)

add_executable(mecanum_control src/mecanum_control.cpp)
ament_target_dependencies(mecanum_control ${THIS_PACKAGE_INCLUDE_DEPENDS}
)
target_include_directories(mecanum_control PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(mecanum_control PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(
        TARGETS mecanum_control
        DESTINATION lib/${PROJECT_NAME}
)

install(
        TARGETS ${PROJECT_NAME}
        DESTINATION lib
)
install(
        DIRECTORY include/
        DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
endif()

ament_export_include_directories(
  include
)
ament_export_libraries(
  ${PROJECT_NAME}
)
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

ament_package()
