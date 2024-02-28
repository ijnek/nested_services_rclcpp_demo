cmake_minimum_required(VERSION 3.8)
project(nested_services_rclcpp_demo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)

# Create executable
add_executable(nested_service src/nested_service.cpp)
ament_target_dependencies(nested_service
  rclcpp
  std_srvs)

# Install
install(TARGETS nested_service
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
