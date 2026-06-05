cmake_minimum_required(VERSION 3.8)
project(turtlebot_rkolio_sim)

find_package(ament_cmake REQUIRED)

# Install URDF / xacro files
install(DIRECTORY urdf/
  DESTINATION share/${PROJECT_NAME}/urdf
)

# Install Gazebo SDF models
install(DIRECTORY models/
  DESTINATION share/${PROJECT_NAME}/models
)

# Install world files
install(DIRECTORY worlds/
  DESTINATION share/${PROJECT_NAME}/worlds
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install config files
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# Install scripts (also install as executables so ros2 run finds them)
install(DIRECTORY scripts/
  DESTINATION share/${PROJECT_NAME}/scripts
  USE_SOURCE_PERMISSIONS
)
install(PROGRAMS scripts/vlp16_ring_time_fixer.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
