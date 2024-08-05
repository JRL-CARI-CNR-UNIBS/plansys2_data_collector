cmake_minimum_required(VERSION 3.8)
project(plansys2_data_collector)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Enforce dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(plansys2_msgs REQUIRED)

# Install pyrobosim Python package
ament_python_install_package(
  plansys2_data_collector)

install(PROGRAMS
  plansys2_data_collector/plot_generator.py
  plansys2_data_collector/plan_plot_generator.py
  plansys2_data_collector/action_execution_info_collector.py
  plansys2_data_collector/plan_execution_info_collector.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch data config
  DESTINATION share/${PROJECT_NAME}
)

# Build tests if enabled
# if(BUILD_TESTING)
#   add_subdirectory(test)
# endif()

ament_package()