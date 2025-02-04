cmake_minimum_required(VERSION 3.8)
project(sally_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  config
  launch
  meshes
  rviz
  urdf
  DESTINATION share/${PROJECT_NAME}/
)

# Add tests and test dependencies
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE) # skips copyright check
  set(ament_cmake_cpplint_FOUND TRUE)   # skips cpplint
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
