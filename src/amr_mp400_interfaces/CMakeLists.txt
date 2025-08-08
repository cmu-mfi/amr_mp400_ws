cmake_minimum_required(VERSION 3.8)
project(amr_mp400_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_typesupport_fastrtps_c REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SetFlag.srv"
  "srv/GroundTruth.srv"
  DEPENDENCIES geometry_msgs
)

# Add these lines to ensure proper type support generation
ament_export_dependencies(rosidl_default_runtime)
ament_export_dependencies(rosidl_typesupport_c)
ament_export_dependencies(rosidl_typesupport_fastrtps_c)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()