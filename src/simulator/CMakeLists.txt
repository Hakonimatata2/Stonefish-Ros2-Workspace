cmake_minimum_required(VERSION 3.8)
project(simulator)

find_package(ament_cmake REQUIRED)

# Installer launch-, scenario- og datafiler til share/
install(DIRECTORY launch scenarios data
  DESTINATION share/${PROJECT_NAME}
)

ament_package()