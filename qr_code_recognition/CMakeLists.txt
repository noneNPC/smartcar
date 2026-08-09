cmake_minimum_required(VERSION 3.8)
project(qr_code_recognition)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(origincar_msg REQUIRED)

# 添加可执行程序
add_executable(qr_code_recognition_node src/qr_code_recognition.cpp)

# 链接依赖库
ament_target_dependencies(qr_code_recognition_node
  rclcpp
  sensor_msgs
  std_msgs
  cv_bridge
  OpenCV
  ament_index_cpp
  origincar_msg
)

# 安装可执行文件
install(TARGETS
  qr_code_recognition_node
  DESTINATION lib/${PROJECT_NAME}
)

# 安装模型文件夹
install(DIRECTORY
  model launch
  DESTINATION share/${PROJECT_NAME}
)

# 安装launch、config等
# install(DIRECTORY launch config
#   DESTINATION share/${PROJECT_NAME}
# )

# 测试相关
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
