cmake_minimum_required(VERSION 3.8)
project(parlam_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)

set(msg_files
  "srv/Input.srv"
  "srv/Llm.srv"
  "srv/Output.srv"
  "action/Llm.action"
)
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES builtin_interfaces std_msgs rcl_interfaces
  ADD_LINTER_TESTS
)

install(
  FILES parlam_msg_mappings.yaml
  DESTINATION share/${PROJECT_NAME})

ament_export_dependencies(rosidl_default_runtime)
ament_package()


