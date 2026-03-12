cmake_minimum_required(VERSION 3.0.2)
project(astar)

## 必须开启 C++14 或 17 以支持较新的 PCL 和 Eigen
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## [核心修改]：清理了 pcl_detection、vision_msgs 等废弃依赖
## 新增了 sensor_msgs 和 pcl_conversions
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs         # 处理 PointCloud2
  geometry_msgs
  mavros_msgs
  visualization_msgs  
  nav_msgs            
  tf                  
  livox_ros_driver
  pcl_ros             
  pcl_conversions     # PCL 与 ROS 消息互转库
)

find_package(PCL REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp rospy std_msgs sensor_msgs geometry_msgs mavros_msgs visualization_msgs nav_msgs tf livox_ros_driver pcl_ros pcl_conversions
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

## 链接 PCL 和 Catkin 库
add_executable(astar 
  src/astar.cpp
  src/ring_crossing.cpp
)

target_link_libraries(astar 
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES} 
)