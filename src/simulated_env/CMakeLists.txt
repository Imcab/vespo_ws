cmake_minimum_required(VERSION 3.8)
project(environment_description)

find_package(ament_cmake REQUIRED)

# Data-only package: Gazebo models and worlds, no code to compile.
install(
  DIRECTORY models worlds
  DESTINATION share/${PROJECT_NAME}
)

# Prepends the model and world folders to GZ_SIM_RESOURCE_PATH on sourcing,
# so model:// URIs and world names resolve without manual exports.
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.dsv.in")

ament_package()
