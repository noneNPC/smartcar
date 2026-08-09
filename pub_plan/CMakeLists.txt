cmake_minimum_required(VERSION 3.8)
project(pub_plan)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1. 寻找 ROS 2 及第三方依赖包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(origincar_msg REQUIRED)

# 2. 生成可执行文件
add_executable(pub_plan src/pub_plan.cpp)

# 3. 关联 ROS 2 依赖项
ament_target_dependencies(pub_plan
  rclcpp
  rclcpp_action
  nav2_msgs
  geometry_msgs
  sensor_msgs
  tf2_ros
  origincar_msg
  tf2_geometry_msgs
)

# 4. 显式链接 nlohmann_json
target_link_libraries(pub_plan nlohmann_json::nlohmann_json)

# 5. 安装目标与资源目录
install(TARGETS pub_plan
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch path config
  DESTINATION share/${PROJECT_NAME}
)

# 6. 代码规范测试
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()