#ifndef NEW_DETECT_OBS_H
#define NEW_DETECT_OBS_H

#include <ros/ros.h>
#include <Eigen/Dense>
#include <vector>
#include <cmath>
#include <nav_msgs/Odometry.h>
// 引入新包的消息类型
#include <pcl_detection/ObjectDetectionResult.h>
#include <pcl_detection/DetectedObject.h>

// 全局变量声明 (保持与 astar.cpp 兼容)
extern float target_x;
extern float target_y;
extern nav_msgs::Odometry local_pos;
extern float if_debug;

// 障碍物结构体 (保持不变)
struct Obstacle
{
    int id;
    Eigen::Vector2f position; // 障碍物中心 (x, y)
    float radius;             // 障碍物半径
};

// 全局障碍物列表 (A* 读取这个)
std::vector<Obstacle> obstacles;

// 回调函数：接收 pcl_detection 的结果并转换
void detection_cb_wrapper(const pcl_detection::ObjectDetectionResult::ConstPtr &msg)
{
    // 1. 清空旧数据 (pcl_detection 发来的是当前帧的完整列表)
    obstacles.clear();

    // 2. 遍历检测到的物体
    int obs_id = 0;
    for (const auto &obj : msg->objects)
    {
        // === 类型 1: 圆柱 (Cylinder) / 类型 2: 圆环 (Circle) ===
        // 直接作为一个圆形障碍物处理
        if (obj.type == 1 || obj.type == 2)
        {
            Obstacle obs;
            obs.id = obs_id++;
            obs.position = Eigen::Vector2f(obj.position.x, obj.position.y);
            // 给半径一点点膨胀余量，或者直接用检测到的半径
            obs.radius = obj.radius;

            // 简单的距离过滤 (太远的不要)
            Eigen::Vector2f drone_pos(local_pos.pose.pose.position.x, local_pos.pose.pose.position.y);
            if ((obs.position - drone_pos).norm() < 10.0)
            {
                obstacles.push_back(obs);
            }
        }
        // === 类型 0: 墙面 (Wall) ===
        // A* 的 Obstacle 结构体是圆形的，无法直接表示长墙。
        // 工程技巧：将墙体“离散化”为一排小圆柱。
        else if (obj.type == 0)
        {
            // 获取法向量 (nx, ny)
            if (obj.plane_coeffs.size() < 3)
                continue;
            double nx = obj.plane_coeffs[0];
            double ny = obj.plane_coeffs[1];
            // 归一化法向量
            double norm = std::sqrt(nx * nx + ny * ny);
            if (norm < 1e-3)
                continue;
            nx /= norm;
            ny /= norm;

            // 墙的切线方向 (旋转90度: -ny, nx)
            double tx = -ny;
            double ty = nx;

            // 墙的中心
            double cx = obj.position.x;
            double cy = obj.position.y;

            // 墙的宽度的一半
            double half_width = obj.width / 2.0;

            // 离散化步长 (每隔 0.2m 放一个障碍点)
            double step = 0.2;
            int steps = std::ceil(half_width / step);

            for (int i = -steps; i <= steps; ++i)
            {
                Obstacle wall_pt;
                wall_pt.id = obs_id++;
                // 沿切线方向排布点
                wall_pt.position.x = cx + tx * (i * step);
                wall_pt.position.y = cy + ty * (i * step);
                wall_pt.radius = 0.15; // 墙体厚度模拟
                obstacles.push_back(wall_pt);
            }
        }
    }

    // 调试日志
    if (if_debug > 0.5)
    {
        ROS_INFO_THROTTLE(1.0, "[Bridge] 收到检测结果: %lu 个物体 -> 转换后 %lu 个障碍点",
                          msg->objects.size(), obstacles.size());
    }
}

// 兼容旧接口的空函数 (因为 main 里可能订阅了 livox，现在不需要了)
// 我们将在 main 中修改订阅者，但这函数留着防止编译报错
void livox_cb_wrapper(const livox_ros_driver::CustomMsg::ConstPtr &msg) {}

#endif