cmake_minimum_required(VERSION 3.8)
project(panda_gazebo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(octomap REQUIRED)
find_package(ament_lint_auto REQUIRED)

# 安装文件夹，如launch、config和rviz
install(
  DIRECTORY
    launch config rviz scripts
  DESTINATION share/${PROJECT_NAME}
)

# 安装Python脚本
install(
  PROGRAMS
    scripts/pointcloud_frame_modifier.py  # 添加你的Python脚本路径
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # 忽略版面检查和cpplint
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

