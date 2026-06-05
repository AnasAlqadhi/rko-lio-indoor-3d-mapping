cmake_minimum_required(VERSION 3.8)
project(turtlebot_rkolio_hardware)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config scripts rviz
  DESTINATION share/${PROJECT_NAME}
  USE_SOURCE_PERMISSIONS
)

install(PROGRAMS scripts/imu_repub.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
