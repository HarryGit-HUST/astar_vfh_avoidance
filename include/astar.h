/*******************************************************************************
astar.h - 无人机智能避障系统核心头文件
============================================================================
系统架构：三层防御体系
L1: A* 全局规划（战略层） - 生成初始路径
L2: 走廊生成器（战术层） - 将离散路径转换为平滑走廊（B-spline + 曲率自适应）
L3: VFH+ 避障（执行层） - 局部避障 + 走廊软约束融合 + 异常检测
异常处理内嵌设计：
• 振荡检测：位置标准差<0.25m + 角度变化>90° → 触发重规划
• 目标不可达：连续 5 帧目标方向被阻挡 → 触发重规划
• 视野变化：检测到新大型障碍物 (半径>0.8m) → 触发重规划
• 振荡恢复：切线方向逃逸策略（40% 最大速度）
工程规范：
• 无文件级全局变量：所有中间数据置于函数栈/静态局部变量
• 命名冲突规避：MAVROS 连接状态 → mavros_connection_state
• 任务状态机：mission_num（高层）+ avoidance_state（mission2 内部）
• 参数驱动：全部行为由 yaml 配置控制，无需改代码
坐标系约定：
• 世界坐标系：Gazebo 全局坐标（原点=起飞点）
• 相对坐标系：任务目标点（相对起飞点，单位：米）
• 机体坐标系：无人机前向为 X+，右向为 Y+
版本：2.0 (2026-02-11)
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
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>

using namespace std;

// ============================================================================
// 全局常量定义
// ============================================================================
/** 飞行高度（米）- 所有任务的统一巡航高度 */
#define ALTITUDE 0.7f

/** Mavros 位置控制指令（全局变量，由各函数写入，main 循环发布） */
extern mavros_msgs::PositionTarget setpoint_raw;

/** 无人机历史位置（二维世界坐标系，单位：米） */
extern Eigen::Vector2f current_pos;

/** 无人机历史速度（二维世界坐标系，单位：米/秒） */
extern Eigen::Vector2f current_vel;

// ============================================================================
// RViz 可视化发布器声明
// ============================================================================
extern ros::Publisher g_obstacle_marker_pub;
extern ros::Publisher g_astar_path_pub;
extern ros::Publisher g_corridor_pub;
extern ros::Publisher g_vfh_grid_pub;

// ============================================================================
// MAVROS 状态
// ============================================================================
/** MAVROS 飞控连接状态（独立变量，避免与任务状态机冲突） */
extern mavros_msgs::State mavros_connection_state;

void state_cb(const mavros_msgs::State::ConstPtr &msg);

// ============================================================================
// 里程计信息
// ============================================================================
extern tf::Quaternion quat;
extern nav_msgs::Odometry local_pos;
extern double roll, pitch, yaw;

extern float init_position_x_take_off;
extern float init_position_y_take_off;
extern float init_position_z_take_off;
extern float init_yaw_take_off;
extern bool flag_init_position;

void local_pos_cb(const nav_msgs::Odometry::ConstPtr &msg);

// ============================================================================
// 位置巡航控制
// ============================================================================
extern float mission_pos_cruise_last_position_x;
extern float mission_pos_cruise_last_position_y;
extern float mission_cruise_timeout;
extern ros::Time mission_cruise_start_time;
extern bool mission_cruise_timeout_flag;
extern bool mission_pos_cruise_flag;

bool mission_pos_cruise(float x, float y, float z, float target_yaw, float error_max);

// ============================================================================
// 精确降落控制
// ============================================================================
extern float precision_land_init_position_x;
extern float precision_land_init_position_y;
extern bool precision_land_init_position_flag;
extern bool hovor_done;
extern bool land_done;
extern ros::Time precision_land_last_time;

bool precision_land(float err_max);

// ============================================================================
// 2D 栅格地图结构体（A*规划使用）
// ============================================================================
/**
@brief 2D 栅格地图结构体（用于 A*规划）
@details
栅格尺寸：200×200
分辨率：0.1 米/栅格
覆盖范围：[-5,5] 米 × [-5,5] 米（以起飞点为中心）
栅格值：0=自由，1~99=未知，100=障碍物
@note 障碍物膨胀采用圆形膨胀，半径=无人机半径 + 安全裕度
*/
struct OccupancyGrid2D
{
    uint8_t cells[200][200]; // 栅格值 [0,100]
    float resolution;        // 栅格分辨率（米）
    float origin_x;          // 地图原点 X（世界坐标系）
    float origin_y;          // 地图原点 Y（世界坐标系）

    /**
    @brief 构造函数：初始化地图参数
    @details
    分辨率：0.1 米/栅格
    原点：(-5.0, -5.0) → 覆盖 [-5,5] 米范围
    初始状态：全自由空间（cells=0）
    */
    OccupancyGrid2D()
    {
        resolution = 0.1f;
        origin_x = -5.0f;
        origin_y = -5.0f;
        for (int i = 0; i < 200; ++i)
            for (int j = 0; j < 200; ++j)
                cells[i][j] = 0;
    }

    /**
    @brief 世界坐标 → 栅格坐标转换
    @param wx 世界 X 坐标（米）
    @param wy 世界 Y 坐标（米）
    @param gx[out] 栅格 X 坐标
    @param gy[out] 栅格 Y 坐标
    @return bool true=转换成功（在地图范围内），false=超出范围
    */
    bool world_to_grid(float wx, float wy, int &gx, int &gy) const
    {
        gx = static_cast<int>((wx - origin_x) / resolution);
        gy = static_cast<int>((wy - origin_y) / resolution);
        return (gx >= 0 && gx < 200 && gy >= 0 && gy < 200);
    }

    /**
    @brief 栅格坐标 → 世界坐标转换
    @param gx 栅格 X 坐标
    @param gy 栅格 Y 坐标
    @param wx[out] 世界 X 坐标（米）
    @param wy[out] 世界 Y 坐标（米）
    */
    void grid_to_world(int gx, int gy, float &wx, float &wy) const
    {
        wx = origin_x + gx * resolution;
        wy = origin_y + gy * resolution;
    }

    /**
    @brief 更新障碍物地图（含衰减机制 + 障碍物膨胀）
    @param obstacles 障碍物列表（来自 PCL 检测）
    @param drone_radius 无人机半径（米）
    @param safety_margin 安全裕度（米）
    @details
    衰减旧障碍物：cells[i][j] -= 2（模拟动态环境）
    投影新障碍物：圆形膨胀（半径=drone_radius+safety_margin）
    @note 由 VFH+ 负责精细避障，A*提供粗糙路径
    */
    void update_with_obstacles(
        const std::vector<Obstacle> &obstacles,
        float drone_radius,
        float safety_margin)
    {
        // 1. 衰减旧障碍物（模拟动态环境）
        for (int i = 0; i < 200; ++i)
        {
            for (int j = 0; j < 200; ++j)
            {
                if (cells[i][j] > 0)
                {
                    cells[i][j] = std::max(static_cast<uint8_t>(0),
                                           static_cast<uint8_t>(cells[i][j] - 2));
                }
            }
        }

        // 2. 障碍物膨胀（圆形，半径=障碍物半径 + 无人机半径 + 安全裕度）
        for (const auto &obs : obstacles)
        {
            int gx, gy;
            if (world_to_grid(obs.position.x(), obs.position.y(), gx, gy))
            {
                // 膨胀半径计算
                float expansion_radius = obs.radius;
                int expansion_cells = static_cast<int>(std::ceil(expansion_radius / resolution));

                // 圆形膨胀
                for (int dx = -expansion_cells; dx <= expansion_cells; ++dx)
                {
                    for (int dy = -expansion_cells; dy <= expansion_cells; ++dy)
                    {
                        int nx = gx + dx;
                        int ny = gy + dy;
                        if (nx >= 0 && nx < 200 && ny >= 0 && ny < 200)
                        {
                            float dist = std::sqrt(dx * dx + dy * dy);
                            if (dist <= expansion_cells)
                            {
                                uint8_t val = static_cast<uint8_t>(100 * (1.0f - dist / expansion_cells));
                                if (val > cells[nx][ny])
                                {
                                    cells[nx][ny] = val;
                                }
                            }
                        }
                    }
                }
            }
        }
    }
};

// ============================================================================
// 核心算法 1：A*全局路径规划
// ============================================================================
int astar_plan(
    const OccupancyGrid2D &grid,
    float start_x, float start_y,
    float goal_x, float goal_y,
    float *path_x, float *path_y,
    int max_points);

// ============================================================================
// 核心算法 2：增量 A*路径规划
// ============================================================================
int incremental_astar_plan(
    const OccupancyGrid2D &grid,
    float start_x, float start_y,
    float goal_x, float goal_y,
    float *path_x, float *path_y,
    int max_points);

// ============================================================================
// 核心算法 3：走廊生成器
// ============================================================================
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
// 核心算法 4：增强 VFH+ 避障
// ============================================================================
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