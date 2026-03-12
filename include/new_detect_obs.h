#ifndef NEW_DETECT_OBS_H
#define NEW_DETECT_OBS_H

#include <ros/ros.h>
#include <Eigen/Dense>
#include <vector>
#include <cmath>
#include <nav_msgs/Odometry.h>
// 替换原有的 #include <pcl_detection/...> 为以下内容
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// ============================================================================
// 全局变量声明
// ============================================================================
extern float target_x;
extern float target_y;
extern nav_msgs::Odometry local_pos;
extern float if_debug;

// ============================================================================
// 核心定义：障碍物类型枚举
// ============================================================================
enum ObsType
{
    WALL = 0,
    RING = 3,
    PILLAR = 4
};

// ============================================================================
// 核心定义：增强版障碍物结构体
// ============================================================================
// 专门留给静态电子围墙使用
struct Obstacle
{
    int id;
    int type;
    Eigen::Vector2f position;
    float radius;
    float width;
    float length;
    float angle;
    std::vector<Eigen::Vector2f> footprint;
};

// 全局障碍物列表容器
extern std::vector<Obstacle> obstacles;
// ==================
// 声明新的全局变量
// ==================
extern pcl::PointCloud<pcl::PointXY>::Ptr current_cloud; // 实时 ROI 点云
extern float current_target_yaw;                         // 飞控锁死的机头目标朝向

#endif