cmake_minimum_required(VERSION 3.8)
project(r2r_spl_test_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# generate interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ArrayTypes.msg"
  "msg/BasicTypes.msg"
  "msg/NestedTypes.msg"
)

# export dependencies
ament_export_dependencies(rosidl_default_runtime)

# test
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
