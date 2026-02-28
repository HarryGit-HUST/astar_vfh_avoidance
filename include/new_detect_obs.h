#ifndef NEW_DETECT_OBS_H
#define NEW_DETECT_OBS_H

#include <ros/ros.h>
#include <Eigen/Dense>
#include <vector>
#include <cmath>
#include <nav_msgs/Odometry.h>
// 引入 PCL 检测包的消息类型
#include <pcl_detection/ObjectDetectionResult.h>
#include <pcl_detection/DetectedObject.h>

// ============================================================================
// 全局变量声明
// ============================================================================
extern float target_x;
extern float target_y;
extern nav_msgs::Odometry local_pos;
extern float if_debug;

// ============================================================================
// 核心定义：障碍物类型枚举 (解决 'CYLINDER' was not declared 报错)
// ============================================================================
enum ObsType
{
    WALL = 0,
    CYLINDER = 1,
    CIRCLE = 2
};

// ============================================================================
// 核心定义：增强版障碍物结构体 (解决 'has no member' 报错)
// ============================================================================
struct Obstacle
{
    int id;
    int type;                 // 障碍物类型 (WALL / CYLINDER)
    Eigen::Vector2f position; // 中心位置 (cx, cy)
    float radius;             // 圆柱半径 或 墙体半厚度

    // 墙体专用属性
    float length; // 墙体长度
    float angle;  // 墙体朝向 (弧度)
};

// 全局障碍物列表容器
extern std::vector<Obstacle> obstacles;

// 回调函数声明
void detection_cb_wrapper(const pcl_detection::ObjectDetectionResult::ConstPtr &msg);
void livox_cb_wrapper(const livox_ros_driver::CustomMsg::ConstPtr &msg);

#endif