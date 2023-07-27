cmake_minimum_required(VERSION 3.8)
project(gear_place)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  rclcpp 
  tf2_kdl
  gear_place_interfaces
  shape_msgs
  OpenCV
  cv_bridge
  Franka
  kdl_parser
)

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(ament_cmake_python REQUIRED)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

add_library(${PROJECT_NAME} SHARED
  src/robot_commander.cpp
)

target_include_directories(${PROJECT_NAME} PRIVATE include)
ament_target_dependencies(${PROJECT_NAME} ${THIS_PACKAGE_INCLUDE_DEPENDS})

# Add robot_commander executable
add_executable(robot_commander_node nodes/robot_commander_node.cpp)
target_include_directories(robot_commander_node PRIVATE include)
target_link_libraries(robot_commander_node
  ${PROJECT_NAME}
)
ament_target_dependencies(robot_commander_node ${THIS_PACKAGE_INCLUDE_DEPENDS})

install(TARGETS robot_commander_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME} SCRIPTS_DESTINATION lib/${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
nodes/gear_place_node.py
nodes/find_object_node.py
nodes/point_cloud_node.py
DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS ${PROJECT_NAME}
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

install(DIRECTORY include/
  DESTINATION include
)


install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_libraries(
  ${PROJECT_NAME}
)
ament_export_include_directories(
  include
)
ament_export_dependencies(
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)
ament_package()
