cmake_minimum_required(VERSION 3.8)
project(spatial_ai_simulation)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY gazebo_models launch worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
