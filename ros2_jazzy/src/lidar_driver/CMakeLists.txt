cmake_minimum_required(VERSION 3.8)
project(lidar_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# 添加第三方库目录
add_subdirectory(third_party/ld_lidar_driver)

add_executable(lidar_node 
    src/lidar_node.cpp 
    src/ldlidar_ls.cpp )
target_include_directories(lidar_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
  ${CMAKE_CURRENT_SOURCE_DIR}/include )
target_compile_features(lidar_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  lidar_node
  "rclcpp"
  "sensor_msgs"
  "geometry_msgs"
)

# 链接依赖 - 关键步骤！
target_link_libraries(lidar_node 
    ld_lidar_driver
)

install(TARGETS lidar_node
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch 
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
endif()

ament_package()
