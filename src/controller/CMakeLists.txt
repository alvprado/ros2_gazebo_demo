cmake_minimum_required(VERSION 3.8)
project(controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

# Domain library: pure C++, no ROS dependencies.
add_library(${PROJECT_NAME}_domain
  src/domain/pid_controller.cpp
)
target_include_directories(${PROJECT_NAME}_domain PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(${PROJECT_NAME}_domain PUBLIC cxx_std_17)

# Core library: ROS 2 infrastructure, depends on the domain library.
add_library(${PROJECT_NAME}_core
  src/infrastructure/velocity_controller_node.cpp
)
target_include_directories(${PROJECT_NAME}_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(${PROJECT_NAME}_core PUBLIC cxx_std_17)
target_link_libraries(${PROJECT_NAME}_core PUBLIC ${PROJECT_NAME}_domain)
ament_target_dependencies(${PROJECT_NAME}_core PUBLIC rclcpp geometry_msgs nav_msgs)

# Executable
add_executable(velocity_controller src/velocity_controller_main.cpp)
target_compile_features(velocity_controller PUBLIC cxx_std_17)
target_link_libraries(velocity_controller ${PROJECT_NAME}_core)

install(TARGETS
  ${PROJECT_NAME}_domain
  ${PROJECT_NAME}_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS velocity_controller
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_pid_controller test/test_pid_controller.cpp)
  target_link_libraries(test_pid_controller ${PROJECT_NAME}_domain)
endif()

ament_package()
