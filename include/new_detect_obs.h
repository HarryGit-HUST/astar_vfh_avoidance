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
// 核心定义：障碍物类型枚举
// ============================================================================
enum ObsType
{
    WALL = 0,
    RING = 3,  // 强调：环门必定是3
    PILLAR = 4 // 方柱类型（原CYLINDER）
};

// ============================================================================
// 核心定义：增强版障碍物结构体
// ============================================================================
struct Obstacle
{
    int id;
    int type;                 // 障碍物类型 (WALL / RING / PILLAR)
    Eigen::Vector2f position; // 中心位置 (cx, cy)
    float radius;             // 墙体厚度/圆柱半径

    // 墙体/老尺寸保留
    float width;
    float length;
    float angle;

    // [终极新增]：真实 OBB 在 2D 地图上的多边形投影轮廓 (凸包)
    std::vector<Eigen::Vector2f> footprint;
};

// 全局障碍物列表容器
extern std::vector<Obstacle> obstacles;

// 回调函数声明
void detection_cb_wrapper(const pcl_detection::ObjectDetectionResult::ConstPtr &msg);
void livox_cb_wrapper(const livox_ros_driver::CustomMsg::ConstPtr &msg);

#endif