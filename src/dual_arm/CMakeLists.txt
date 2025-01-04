cmake_minimum_required(VERSION 3.8)
project(dual_arm)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclpy REQUIRED)

# 安装 Python 脚本（整个 scripts 目录）
install(DIRECTORY
  scripts/  # 这会安装 scripts 目录下的所有文件
  DESTINATION lib/${PROJECT_NAME}  # 安装路径
  FILES_MATCHING PATTERN "*.py"  # 仅匹配 .py 文件
)



# 安装其他文件（launch 文件、配置文件等）
install(
  DIRECTORY
    launch
    config
    rviz
  DESTINATION share/${PROJECT_NAME}
)

# 如果需要构建测试，进行 lint 等检查
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
