cmake_minimum_required(VERSION 3.8)
project(aljnuho_bt)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(ament_index_cpp REQUIRED)

include_directories(${behaviortree_cpp_INCLUDE_DIRS})

# add_executable(bt_hello src/bt_hello.cpp)
# target_link_libraries(bt_hello behaviortree_cpp::behaviortree_cpp)

find_package(robotiq2f_interfaces REQUIRED)
add_executable(bt_handover_ros2 src/bt_handover_ros2.cpp)
ament_target_dependencies(bt_handover_ros2 ament_index_cpp rclcpp robotiq2f_interfaces)
target_link_libraries(bt_handover_ros2 behaviortree_cpp::behaviortree_cpp)

# file(GLOB SRC_FILES src/*.cpp)
# foreach(SRC_FILE ${SRC_FILES})
#   get_filename_component(EXECUTABLE_NAME ${SRC_FILE} NAME_WE)

#   add_executable(${EXECUTABLE_NAME} ${SRC_FILE})
#   target_link_libraries(${EXECUTABLE_NAME}
#     behaviortree_cpp::behaviortree_cpp
#     ament_index_cpp::ament_index_cpp
#   )
# endforeach()

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/)

install(TARGETS
 bt_handover_ros2
  DESTINATION lib/${PROJECT_NAME})

ament_package()
