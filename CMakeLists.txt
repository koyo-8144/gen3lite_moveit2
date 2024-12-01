cmake_minimum_required(VERSION 3.8)
project(kinova_moveit)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(kinova_moveit_test src/kinova_moveit_test.cpp) # added
add_executable(openvla_test src/openvla_test.cpp) # added
add_executable(kinova_moveit_openvla src/kinova_moveit_openvla.cpp) # added
add_executable(kinova_moveit_openvla_sim src/kinova_moveit_openvla_sim.cpp) # added
add_executable(end_effector_moveit src/end_effector_moveit.cpp) # added
add_executable(kinova_moveit_openvla_endeffector_sim src/kinova_moveit_openvla_endeffector_sim.cpp) # added
add_executable(kinova_no_moveit_openvla src/kinova_no_moveit_openvla.cpp) # added

# added
target_include_directories(kinova_moveit_test PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(kinova_moveit_test PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(openvla_test PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(openvla_test PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(kinova_moveit_openvla PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(kinova_moveit_openvla PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(kinova_moveit_openvla_sim PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(kinova_moveit_openvla_sim PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(end_effector_moveit PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(end_effector_moveit PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(kinova_moveit_openvla_endeffector_sim PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(kinova_moveit_openvla_endeffector_sim PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(kinova_no_moveit_openvla PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(kinova_no_moveit_openvla PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

ament_target_dependencies(
  kinova_moveit_test
  "moveit_ros_planning_interface"
  "rclcpp"
)
# added
ament_target_dependencies(
  openvla_test
  "moveit_ros_planning_interface"
  "rclcpp"
)
# added
ament_target_dependencies(
  kinova_moveit_openvla
  "moveit_ros_planning_interface"
  "rclcpp"
)
# added
ament_target_dependencies(
  kinova_moveit_openvla_sim
  "moveit_ros_planning_interface"
  "rclcpp"
)
# added
ament_target_dependencies(
  end_effector_moveit
  "moveit_ros_planning_interface"
  "rclcpp"
)
# added
ament_target_dependencies(
  kinova_moveit_openvla_endeffector_sim
  "moveit_ros_planning_interface"
  "rclcpp"
)
# added
ament_target_dependencies(
  kinova_no_moveit_openvla
  "moveit_ros_planning_interface"
  "rclcpp"
)


install(TARGETS kinova_moveit_test
  DESTINATION lib/${PROJECT_NAME})
# added
install(TARGETS openvla_test
  DESTINATION lib/${PROJECT_NAME})
# added
install(TARGETS kinova_moveit_openvla
DESTINATION lib/${PROJECT_NAME})
# added
install(TARGETS kinova_moveit_openvla_sim
DESTINATION lib/${PROJECT_NAME})
# added
install(TARGETS end_effector_moveit
DESTINATION lib/${PROJECT_NAME})
# added
install(TARGETS kinova_moveit_openvla_endeffector_sim
DESTINATION lib/${PROJECT_NAME})
# added
install(TARGETS kinova_no_moveit_openvla
DESTINATION lib/${PROJECT_NAME})

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
endif()

ament_package()
