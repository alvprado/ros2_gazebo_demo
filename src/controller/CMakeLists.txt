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
find_package(std_msgs REQUIRED)

# Domain library: pure C++, no ROS dependencies.
add_library(${PROJECT_NAME}_domain
  src/domain/low_pass_filter.cpp
  src/domain/targets_generator.cpp
)
target_include_directories(${PROJECT_NAME}_domain PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(${PROJECT_NAME}_domain PUBLIC cxx_std_23)

# Core library: ROS 2 infrastructure for the controller node.
add_library(${PROJECT_NAME}_core
  src/infrastructure/controller_node.cpp
)
target_include_directories(${PROJECT_NAME}_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(${PROJECT_NAME}_core PUBLIC cxx_std_23)
target_link_libraries(${PROJECT_NAME}_core PUBLIC ${PROJECT_NAME}_domain)
ament_target_dependencies(${PROJECT_NAME}_core PUBLIC rclcpp geometry_msgs nav_msgs std_msgs)

# Targets generator library: ROS 2 infrastructure for the targets generator node.
add_library(${PROJECT_NAME}_targets_generator_core
  src/infrastructure/targets_generator_node.cpp
)
target_include_directories(${PROJECT_NAME}_targets_generator_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(${PROJECT_NAME}_targets_generator_core PUBLIC cxx_std_23)
target_link_libraries(${PROJECT_NAME}_targets_generator_core PUBLIC ${PROJECT_NAME}_domain)
ament_target_dependencies(${PROJECT_NAME}_targets_generator_core PUBLIC rclcpp std_msgs)

# Executables
add_executable(controller src/controller_main.cpp)
target_compile_features(controller PUBLIC cxx_std_23)
target_link_libraries(controller ${PROJECT_NAME}_core)

add_executable(targets_generator src/targets_generator_main.cpp)
target_compile_features(targets_generator PUBLIC cxx_std_23)
target_link_libraries(targets_generator ${PROJECT_NAME}_targets_generator_core)

install(TARGETS
  ${PROJECT_NAME}_domain
  ${PROJECT_NAME}_core
  ${PROJECT_NAME}_targets_generator_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  controller
  targets_generator
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_pid_controller test/test_pid_controller.cpp)
  target_link_libraries(test_pid_controller ${PROJECT_NAME}_domain)
  target_compile_features(test_pid_controller PUBLIC cxx_std_23)

  ament_add_gtest(test_curvature_controller test/test_curvature_controller.cpp)
  target_link_libraries(test_curvature_controller ${PROJECT_NAME}_domain)
  target_compile_features(test_curvature_controller PUBLIC cxx_std_23)
endif()

ament_package()
