/*******************************************************************************
astar.h - 无人机智能避障系统核心头文件
============================================================================
系统架构：三层防御体系
L1: A*全局规划（战略层） - 生成初始路径
L2: 走廊生成器（战术层） - 将离散路径转换为平滑走廊
L3: VFH+ 避障（执行层） - 局部避障 + 走廊软约束融合 + 异常检测

<第 1 次修改> 动态地图改为全局地图，分辨率调整为 0.15m
<第 2 次修改> 优化振荡检测阈值，避免低速误判
<第 3 次修改> 增加调试 ROS_INFO，提高可溯源性
<第 4 次修改> 参数外置到 yaml，增加物理含义注释

版本：3.0 (2026-02-14)
作者：智能避障系统
******************************************************************************/
#ifndef ASTAR_H
#define ASTAR_H

#include <string>
#include <vector>
#include "new_detect_obs.h"
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <std_msgs/Bool.h>
#include <mavros_msgs/PositionTarget.h>
#include <cmath>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <mavros_msgs/CommandLong.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int32.h>
#include <livox_ros_driver/CustomMsg.h>
#include <eigen3/Eigen/Dense>
#include <queue>
#include <unordered_map>
#include <unordered_set>

using namespace std;

// ============================================================================
// 全局常量定义
// ============================================================================
/** 飞行高度（米）- 所有任务的统一巡航高度 */
#define ALTITUDE 0.7f

/** Mavros 位置控制指令（全局变量，由各函数写入，main 循环发布） */
mavros_msgs::PositionTarget setpoint_raw;

/** 无人机历史位置（二维世界坐标系，单位：米） */
Eigen::Vector2f current_pos;

/** 无人机历史速度（二维世界坐标系，单位：米/秒） */
Eigen::Vector2f current_vel;

// ============================================================================
// 可配置参数（从 yaml 读取，具有物理含义）
// ============================================================================
/** 全局地图尺寸（米）- 地图覆盖范围 */
float GLOBAL_MAP_SIZE = 20.0f;

/** 全局地图分辨率（米/栅格）- 分辨率越低，地图越精细但计算量越大 */
float GLOBAL_MAP_RESOLUTION = 0.15f;

/** 振荡检测位置阈值（米）- 低于此值且角度变化大时判定为振荡 */
float OSCILLATION_POS_THRESHOLD = 0.15f;

/** 振荡检测角度阈值（度）- 高于此值且位置稳定时判定为振荡 */
float OSCILLATION_ANGLE_THRESHOLD = 120.0f;

/** 振荡检测时间窗口（秒）- 历史数据时间窗口 */
float OSCILLATION_TIME_WINDOW = 3.0f;

/** 振荡恢复持续时间（秒）- 振荡恢复策略持续时间 */
float OSCILLATION_RECOVERY_DURATION = 2.0f;

/** A*安全裕度（米）- A*规划时的安全裕度，0 表示无额外裕度 */
float ASTAR_SAFETY_MARGIN = 0.0f;

/** 重规划冷却时间（秒）- 两次重规划之间的最小时间间隔 */
//float REPLAN_COOLDOWN = 5.0f;

/** 走廊基础宽度（米）- 走廊的基础宽度 */
float CORRIDOR_BASE_WIDTH = 0.8f;

/** 走廊急弯加宽系数 - 急弯时走廊宽度放大倍数 */
float CORRIDOR_CURVATURE_SCALE = 1.2f;

/** 走廊曲率阈值（1/米）- 超过此曲率视为急弯 */
float CORRIDOR_CURVATURE_THRESHOLD = 0.4f;

// ============================================================================
// 函数 1：MAVROS 连接状态回调
// ============================================================================
/** MAVROS 飞控连接状态（独立变量，避免与任务状态机冲突） */
mavros_msgs::State mavros_connection_state;

/**
 * @brief MAVROS 状态回调函数
 * @param msg MAVROS 状态消息指针
 */
void state_cb(const mavros_msgs::State::ConstPtr &msg);
void state_cb(const mavros_msgs::State::ConstPtr &msg)
{
    mavros_connection_state = *msg;
}

// ============================================================================
// 函数 2：里程计信息回调
// ============================================================================
/** 四元数（临时变量，用于姿态转换） */
tf::Quaternion quat;

/** 里程计原始数据（全局变量，供其他函数读取） */
nav_msgs::Odometry local_pos;

/** 欧拉角（roll/pitch/yaw，单位：弧度） */
double roll, pitch, yaw;

/** 起飞点世界坐标（X/Y/Z，单位：米） */
float init_position_x_take_off = 0;
float init_position_y_take_off = 0;
float init_position_z_take_off = 0;

/** 起飞点航向角（单位：弧度） */
float init_yaw_take_off = 0;

/** 起飞点初始化标志 */
bool flag_init_position = false;

/**
 * @brief 里程计回调函数
 * @param msg 里程计消息指针
 */
void local_pos_cb(const nav_msgs::Odometry::ConstPtr &msg);
void local_pos_cb(const nav_msgs::Odometry::ConstPtr &msg)
{
    local_pos = *msg;
    tf::quaternionMsgToTF(local_pos.pose.pose.orientation, quat);
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    current_pos = Eigen::Vector2f(local_pos.pose.pose.position.x, local_pos.pose.pose.position.y);

    // 机体速度→世界速度转换
    tf::Vector3 body_vel(local_pos.twist.twist.linear.x, local_pos.twist.twist.linear.y, local_pos.twist.twist.linear.z);
    tf::Matrix3x3 rot_matrix(quat);
    tf::Vector3 world_vel = rot_matrix * body_vel;
    current_vel = Eigen::Vector2f(world_vel.x(), world_vel.y());

    // 首次高度>0.1m 时记录起飞点
    if (flag_init_position == false && (local_pos.pose.pose.position.z > 0.1))
    {
        init_position_x_take_off = local_pos.pose.pose.position.x;
        init_position_y_take_off = local_pos.pose.pose.position.y;
        init_position_z_take_off = local_pos.pose.pose.position.z;
        init_yaw_take_off = yaw;
        flag_init_position = true;
        ROS_INFO("[ODOM] 起飞点已初始化：(%.2f, %.2f, %.2f)",
                 init_position_x_take_off, init_position_y_take_off, init_position_z_take_off);
    }
}

// ============================================================================
// 函数 3：位置巡航控制
// ============================================================================
/** 上次巡航目标位置 */
float mission_pos_cruise_last_position_x = 0;
float mission_pos_cruise_last_position_y = 0;

/** 巡航超时阈值（秒） */
float mission_cruise_timeout = 180.0f;

/** 任务开始时间 */
ros::Time mission_cruise_start_time;

/** 超时标志 */
bool mission_cruise_timeout_flag = false;

/** 任务标志 */
bool mission_pos_cruise_flag = false;

/**
 * @brief 位置巡航控制函数
 * @return bool true=到达目标点，false=未到达
 */
bool mission_pos_cruise(float x, float y, float z, float target_yaw, float error_max);
bool mission_pos_cruise(float x, float y, float z, float target_yaw, float error_max)
{
    if (mission_pos_cruise_flag == false)
    {
        mission_pos_cruise_last_position_x = local_pos.pose.pose.position.x;
        mission_pos_cruise_last_position_y = local_pos.pose.pose.position.y;
        mission_pos_cruise_flag = true;
        mission_cruise_start_time = ros::Time::now();
        mission_cruise_timeout_flag = false;
    }

    ros::Duration elapsed_time = ros::Time::now() - mission_cruise_start_time;
    if (elapsed_time.toSec() > mission_cruise_timeout && !mission_cruise_timeout_flag)
    {
        ROS_WARN("[巡航超时] 已耗时%.1f 秒，强制切换下一个任务！", elapsed_time.toSec());
        mission_cruise_timeout_flag = true;
        mission_pos_cruise_flag = false;
        return true;
    }

    setpoint_raw.type_mask = 8 + 16 + 32 + 64 + 128 + 256 + 512 + 2048;
    setpoint_raw.coordinate_frame = 1;
    setpoint_raw.position.x = x + init_position_x_take_off;
    setpoint_raw.position.y = y + init_position_y_take_off;
    setpoint_raw.position.z = z + init_position_z_take_off;
    setpoint_raw.yaw = target_yaw;

    if (fabs(local_pos.pose.pose.position.x - x - init_position_x_take_off) < error_max &&
        fabs(local_pos.pose.pose.position.y - y - init_position_y_take_off) < error_max &&
        fabs(local_pos.pose.pose.position.z - z - init_position_z_take_off) < error_max &&
        fabs(yaw - target_yaw) < 0.1)
    {
        ROS_INFO("到达目标点，巡航点任务完成");
        mission_cruise_timeout_flag = false;
        mission_pos_cruise_flag = false;
        return true;
    }
    return false;
}

// ============================================================================
// 函数 4：精确降落控制
// ============================================================================
/** 降落起始位置 */
float precision_land_init_position_x = 0;
float precision_land_init_position_y = 0;

/** 降落起始标志 */
bool precision_land_init_position_flag = false;

/** 悬停完成标志 */
bool hovor_done = false;

/** 降落完成标志 */
bool land_done = false;

/** 阶段切换时间戳 */
ros::Time precision_land_last_time;

/**
 * @brief 精确降落控制函数
 * @return bool true=降落完成，false=降落中
 */
bool precision_land(float err_max);
bool precision_land(float err_max)
{
    if (!precision_land_init_position_flag)
    {
        precision_land_init_position_x = local_pos.pose.pose.position.x;
        precision_land_init_position_y = local_pos.pose.pose.position.y;
        precision_land_last_time = ros::Time::now();
        precision_land_init_position_flag = true;
    }

    if (fabs(local_pos.pose.pose.position.x - precision_land_init_position_x) < err_max / 2 &&
            fabs(local_pos.twist.twist.linear.x) < err_max / 10 &&
            fabs(local_pos.pose.pose.position.y - precision_land_init_position_y) < err_max / 2 &&
            fabs(local_pos.twist.twist.linear.y) < err_max / 10 ||
        ros::Time::now() - precision_land_last_time > ros::Duration(10.0))
    {
        hovor_done = true;
        precision_land_last_time = ros::Time::now();
    }

    if (!land_done && hovor_done && (fabs(local_pos.pose.pose.position.z - init_position_z_take_off) < err_max / 5 || ros::Time::now() - precision_land_last_time > ros::Duration(5.0)))
    {
        land_done = true;
        precision_land_last_time = ros::Time::now();
    }

    if (land_done && ros::Time::now() - precision_land_last_time > ros::Duration(2.0))
    {
        ROS_INFO("Precision landing complete.");
        precision_land_init_position_flag = false;
        hovor_done = false;
        land_done = false;
        return true;
    }

    setpoint_raw.position.x = precision_land_init_position_x;
    setpoint_raw.position.y = precision_land_init_position_y;

    if (!land_done && !hovor_done)
    {
        setpoint_raw.position.z = ALTITUDE;
        ROS_INFO("悬停中");
    }
    else if (!land_done)
    {
        setpoint_raw.position.z = (local_pos.pose.pose.position.z + 0.15) * 0.75 - 0.15;
        ROS_INFO("降落中");
    }
    else
    {
        setpoint_raw.position.z = local_pos.pose.pose.position.z - 0.02;
        ROS_INFO("稳定中");
    }

    setpoint_raw.type_mask = 8 + 16 + 32 + 64 + 128 + 256 + 512;
    setpoint_raw.coordinate_frame = 1;
    return false;
}

// ============================================================================
// 辅助结构：2D 栅格地图（A*规划使用）
// ============================================================================
/**
 * @brief 2D 栅格地图结构体（用于 A*规划）
 *
 * <第 1 次修改> 动态地图改为全局地图，分辨率调整为 0.15m
 */
struct OccupancyGrid2D
{
    uint8_t cells[200][200]; // <第 1 次修改> 地图尺寸从 100x100 改为 200x200
    float resolution;
    float origin_x;
    float origin_y;
    int map_size; // <第 1 次修改> 地图尺寸

    /**
     * @brief 构造函数：初始化地图参数
     *
     * <第 1 次修改> 使用全局地图参数初始化
     */
    OccupancyGrid2D()
    {
        resolution = GLOBAL_MAP_RESOLUTION;             // <第 1 次修改> 使用可配置分辨率
        map_size = (int)(GLOBAL_MAP_SIZE / resolution); // <第 1 次修改> 根据分辨率计算地图尺寸
        origin_x = -GLOBAL_MAP_SIZE / 2.0f;             // <第 1 次修改> 原点在地图中心
        origin_y = -GLOBAL_MAP_SIZE / 2.0f;

        for (int i = 0; i < map_size; ++i)
            for (int j = 0; j < map_size; ++j)
                cells[i][j] = 0;

        ROS_INFO("[MAP] 全局地图初始化：尺寸=%dx%d, 分辨率=%.2fm, 覆盖范围=%.1fx%.1fm",
                 map_size, map_size, resolution, GLOBAL_MAP_SIZE, GLOBAL_MAP_SIZE);
    }

    /**
     * @brief 世界坐标 → 栅格坐标转换
     */
    bool world_to_grid(float wx, float wy, int &gx, int &gy) const
    {
        gx = static_cast<int>((wx - origin_x) / resolution);
        gy = static_cast<int>((wy - origin_y) / resolution);
        return (gx >= 0 && gx < map_size && gy >= 0 && gy < map_size);
    }

    /**
     * @brief 栅格坐标 → 世界坐标转换
     */
    void grid_to_world(int gx, int gy, float &wx, float &wy) const
    {
        wx = origin_x + gx * resolution;
        wy = origin_y + gy * resolution;
    }

    /**
     * @brief 更新障碍物地图
     *
     * <第 4 次修改> 移除安全裕度，仅标记障碍物本身
     */
    void update_with_obstacles(
        const std::vector<Obstacle> &obstacles,
        float drone_radius,
        float safety_margin)
    {
        // 1. 衰减旧障碍物
        for (int i = 0; i < map_size; ++i)
        {
            for (int j = 0; j < map_size; ++j)
            {
                if (cells[i][j] > 0)
                {
                    cells[i][j] = std::max(static_cast<uint8_t>(0),
                                           static_cast<uint8_t>(cells[i][j] - 2));
                }
            }
        }

        // 2. <第 4 次修改> 仅标记障碍物中心，无额外安全裕度
        for (const auto &obs : obstacles)
        {
            int gx, gy;
            if (world_to_grid(obs.position.x(), obs.position.y(), gx, gy))
            {
                cells[gx][gy] = 100;
                ROS_DEBUG("[MAP] 障碍物标记：(%.2f, %.2f) -> 栅格 (%d,%d)",
                          obs.position.x(), obs.position.y(), gx, gy);
            }
        }
    }
};

// ============================================================================
// 核心算法 1：A*全局路径规划
// ============================================================================
/**
 * @brief A*全局路径规划函数
 */
int astar_plan(
    const OccupancyGrid2D &grid,
    float start_x, float start_y,
    float goal_x, float goal_y,
    float *path_x, float *path_y,
    int max_points);

// ============================================================================
// 核心算法 2：走廊生成器
// ============================================================================
/**
 * @brief 走廊生成器
 */
int generate_corridor(
    const float *astar_path_x,
    const float *astar_path_y,
    int num_points,
    float base_width,
    float *corridor_x,
    float *corridor_y,
    float *corridor_width,
    int max_size);

// ============================================================================
// 核心算法 3：增强 VFH+ 避障
// ============================================================================
/**
 * @brief 增强 VFH+ 避障函数
 *
 * <第 2 次修改> 优化振荡检测阈值
 * <第 4 次修改> 参数从 yaml 读取
 */
bool vfh_plus_with_corridor(
    float target_x_rel,
    float target_y_rel,
    float target_yaw,
    float uav_radius,
    float safe_margin,
    float max_speed,
    float min_safe_distance,
    const float *corridor_x,
    const float *corridor_y,
    const float *corridor_width,
    int corridor_size,
    bool enable_corridor,
    bool &out_need_replan);

#endif // ASTAR_H