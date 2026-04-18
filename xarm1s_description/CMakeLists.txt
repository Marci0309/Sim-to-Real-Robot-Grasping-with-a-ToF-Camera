cmake_minimum_required(VERSION 3.8)
project(xarm1s_description)
find_package(ament_cmake REQUIRED)
install(DIRECTORY urdf meshes launch rviz
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL)
ament_package()
