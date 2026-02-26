cmake_minimum_required(VERSION 3.0.2)
project(astar)

## 必须开启 C++14 或 17 以支持较新的 PCL 和 Eigen
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## 添加缺少的依赖库：visualization_msgs, nav_msgs, tf, pcl_ros
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  mavros_msgs
  visualization_msgs  # 新增：用于发布 Marker
  nav_msgs            # 新增：用于发布 Path
  tf                  # 新增：用于坐标变换
  livox_ros_driver
  pcl_ros             # 新增：用于点云处理
  pcl_conversions     # 新增
  pcl_detection
)

find_package(PCL REQUIRED)

catkin_package(
  INCLUDE_DIRS include
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

## 链接 PCL 和 Catkin 库
add_executable(astar src/astar.cpp)
target_link_libraries(astar 
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES} 
)