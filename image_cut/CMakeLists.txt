cmake_minimum_required(VERSION 3.5)
project(image_cut)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ai_msgs REQUIRED)
find_package(hbm_img_msgs REQUIRED)
find_package(dnn_node REQUIRED)
find_package(hobot_cv REQUIRED)
find_package(ament_index_cpp REQUIRED)

# 设置默认 hobotcv 库路径
set(HOBOTCV_LIB_INSTALL_PATH ${CMAKE_INSTALL_PREFIX}/lib)
if(hobot_cv_LIBRARIES)
  message("hobot_cv_LIBRARIES is ${hobot_cv_LIBRARIES}")
  string(FIND ${hobot_cv_LIBRARIES} "/" index REVERSE)
  string(SUBSTRING ${hobot_cv_LIBRARIES} 0 ${index} HOBOTCV_LIB_INSTALL_PATH)
  message("update HOBOTCV_LIB_INSTALL_PATH as " ${HOBOTCV_LIB_INSTALL_PATH})
endif()

message("HOBOTCV_LIB_INSTALL_PATH is " ${HOBOTCV_LIB_INSTALL_PATH})

link_directories(
  /usr/hobot/lib
  /usr/lib/
  /usr/lib/hbbpu/
  /usr/lib/hbmedia/
  ${HOBOTCV_LIB_INSTALL_PATH}
)

include_directories(
  include
  ${PROJECT_SOURCE_DIR}
  /usr/include
  /usr/include/dnn
  /usr/hobot/include
)

add_executable(image_cut_node
  src/yolov11_detector.cpp
)

ament_target_dependencies(
  image_cut_node
  rclcpp
  dnn_node
  sensor_msgs
  ai_msgs
  hbm_img_msgs
  hobot_cv
)

# Install executables
install(
  TARGETS image_cut_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  model
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  ${PROJECT_SOURCE_DIR}/launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()