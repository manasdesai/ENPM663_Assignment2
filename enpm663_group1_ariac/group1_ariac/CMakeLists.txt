cmake_minimum_required(VERSION 3.8)
project(group1_ariac)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(ariac_msgs REQUIRED)

# Python
ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS
  script/check_competition_status.py
  script/retrieve_orders.py
  DESTINATION lib/${PROJECT_NAME}
)
ament_package()