cmake_minimum_required(VERSION 3.16)
project(se_mppi_utrap)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY worlds urdf launch config behavior_trees
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
