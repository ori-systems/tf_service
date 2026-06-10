cmake_minimum_required(VERSION 3.8)
project(tf_service)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/CanTransform.srv"
  "srv/LookupTransform.srv"
  DEPENDENCIES builtin_interfaces geometry_msgs tf2_msgs
)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_library(${PROJECT_NAME}_cpp
  src/buffer_client.cpp
  src/buffer_server.cpp
)
target_include_directories(${PROJECT_NAME}_cpp PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME}_cpp "${cpp_typesupport_target}")
ament_target_dependencies(${PROJECT_NAME}_cpp
  builtin_interfaces geometry_msgs rclcpp tf2 tf2_msgs tf2_ros
)

target_compile_features(${PROJECT_NAME}_cpp PUBLIC cxx_std_17)

add_executable(server src/server_main.cpp)
target_link_libraries(server ${PROJECT_NAME}_cpp)
ament_target_dependencies(server rclcpp tf2 tf2_ros)

install(TARGETS ${PROJECT_NAME}_cpp server
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_python_install_package(tf_service)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_cpp)
ament_export_dependencies(
  builtin_interfaces geometry_msgs rclcpp tf2 tf2_msgs tf2_ros rosidl_default_runtime
  ament_cmake_python
)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_package()
