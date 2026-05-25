cmake_minimum_required(VERSION 3.8)
project(pendulum_swing)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Python 安装
ament_python_install_package(${PROJECT_NAME})

# 安装 launch 文件
install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# 安装 URDF 文件
install(
  DIRECTORY urdf/
  DESTINATION share/${PROJECT_NAME}/urdf
)

# 安装配置文件
install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# 安装 Python 脚本（重命名为无 .py 扩展名的可执行文件）
install(PROGRAMS
  pendulum_swing/swing_up_controller.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME swing_up_controller
)

ament_package()
