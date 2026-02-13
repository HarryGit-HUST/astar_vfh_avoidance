/*******************************************************************************
astar.h - 无人机智能避障系统核心头文件
============================================================================
系统架构：三层防御体系
L1: A*全局规划（战略层） - 生成初始路径（动态地图+目标点投影）
L2: 走廊生成器（战术层） - 将离散路径转换为平滑走廊（B-spline + 曲率自适应）
L3: VFH+避障（执行层） - 局部避障 + 走廊软约束融合 + 异常检测
异常处理内嵌设计：
• 振荡检测：位置标准差<0.25m + 角度变化>90° → 触发重规划
• 路径阻塞检测：规划路径被新障碍物占据 → 触发重规划
• 视野变化：检测到新大型障碍物(半径>0.8m) → 触发重规划
工程规范：
• 动态地图：以无人机当前位置为中心，覆盖[-5.0,5.0]米范围
• 目标点投影：超出地图时投影到边界（连线交点）
• 障碍物膨胀：仅使用障碍物半径+无人机半径（无额外安全裕度）
• 路径简化：道格拉斯-普克算法（阈值0.3m）
版本：2.2 (2026-02-14)
作者：智能避障系统
******************************************************************************/
#ifndef ASTAR_H
#define ASTAR_H
#include <string>
#include <vector>
#include "new_detect_obs.h" // PCL障碍物检测接口
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
/** Mavros位置控制指令（全局变量，由各函数写入，main循环发布） */
mavros_msgs::PositionTarget setpoint_raw;
/** 无人机历史位置（二维世界坐标系，单位：米） */
Eigen::Vector2f current_pos;
/** 无人机历史速度（二维世界坐标系，单位：米/秒） */
Eigen::Vector2f current_vel;
// ============================================================================
// 函数1：MAVROS连接状态回调
// ============================================================================
mavros_msgs::State mavros_connection_state;
void state_cb(const mavros_msgs::State::ConstPtr &msg);
void state_cb(const mavros_msgs::State::ConstPtr &msg)
{
    mavros_connection_state = *msg;
}
// ============================================================================
// 函数2：里程计信息回调
// ============================================================================
tf::Quaternion quat;
nav_msgs::Odometry local_pos;
double roll, pitch, yaw;
float init_position_x_take_off = 0;
float init_position_y_take_off = 0;
float init_position_z_take_off = 0;
float init_yaw_take_off = 0;
bool flag_init_position = false;
void local_pos_cb(const nav_msgs::Odometry::ConstPtr &msg);
void local_pos_cb(const nav_msgs::Odometry::ConstPtr &msg)
{
    local_pos = *msg;
    tf::quaternionMsgToTF(local_pos.pose.pose.orientation, quat);
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    current_pos = Eigen::Vector2f(local_pos.pose.pose.position.x, local_pos.pose.pose.position.y);
    tf::Vector3 body_vel(local_pos.twist.twist.linear.x, local_pos.twist.twist.linear.y, local_pos.twist.twist.linear.z);
    tf::Matrix3x3 rot_matrix(quat);
    tf::Vector3 world_vel = rot_matrix * body_vel;
    current_vel = Eigen::Vector2f(world_vel.x(), world_vel.y());
    if (flag_init_position == false && (local_pos.pose.pose.position.z > 0.1))
    {
        init_position_x_take_off = local_pos.pose.pose.position.x;
        init_position_y_take_off = local_pos.pose.pose.position.y;
        init_position_z_take_off = local_pos.pose.pose.position.z;
        init_yaw_take_off = yaw;
        flag_init_position = true;
    }
}
// ============================================================================
// 函数3：位置巡航控制
// ============================================================================
float mission_pos_cruise_last_position_x = 0;
float mission_pos_cruise_last_position_y = 0;
float mission_cruise_timeout = 180.0f;
ros::Time mission_cruise_start_time;
bool mission_cruise_timeout_flag = false;
bool mission_pos_cruise_flag = false;
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
        ROS_WARN("[巡航超时] 已耗时%.1f秒（阈值%.1f秒），强制切换下一个任务！", elapsed_time.toSec(), mission_cruise_timeout);
        mission_cruise_timeout_flag = true;
        mission_pos_cruise_flag = false;
        return true;
    }
    setpoint_raw.type_mask = /*1 + 2 + 4 */ +8 + 16 + 32 + 64 + 128 + 256 + 512 /*+ 1024 */ + 2048;
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
// 函数4：精确降落控制
// ============================================================================
float precision_land_init_position_x = 0;
float precision_land_init_position_y = 0;
bool precision_land_init_position_flag = false;
bool hovor_done = false;
bool land_done = false;
ros::Time precision_land_last_time;
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
    setpoint_raw.type_mask = /*1 + 2 + 4 +*/ 8 + 16 + 32 + 64 + 128 + 256 + 512 /*+ 1024 + 2048*/;
    setpoint_raw.coordinate_frame = 1;
    return false;
}
// ============================================================================
// 辅助结构：2D栅格地图（A*规划使用）
// ============================================================================
/**
@brief 2D栅格地图结构体（用于A*规划）
@details
【动态地图设计】
- 地图始终以无人机当前位置为中心
- 覆盖范围：[-5.0, 5.0]米 × [-5.0, 5.0]米（相对无人机当前位置）
- 栅格尺寸：100×100，分辨率0.1米/栅格
【坐标转换】
  世界坐标(x,y) → 栅格坐标(gx,gy):
    gx = (x - drone_x) / resolution + 50  // 50为中心栅格
    gy = (y - drone_y) / resolution + 50
【障碍物处理】
  膨胀半径 = 障碍物半径 + 无人机半径（无额外安全裕度）
@note 动态地图确保A*始终规划前方5米内路径，避免边界问题
*/
struct OccupancyGrid2D
{
    uint8_t cells[100][100]; // 栅格值 [0,100]
    float resolution;        // 栅格分辨率（米）
    float drone_x;           // 无人机当前位置X（世界坐标系，用于动态地图中心）
    float drone_y;           // 无人机当前位置Y（世界坐标系）

    OccupancyGrid2D()
    {
        resolution = 0.1f;
        drone_x = 0.0f;
        drone_y = 0.0f;
        for (int i = 0; i < 100; ++i)
            for (int j = 0; j < 100; ++j)
                cells[i][j] = 0;
    }

    /**
    @brief 世界坐标 → 栅格坐标转换（动态地图）
    @param wx 世界X坐标（米）
    @param wy 世界Y坐标（米）
    @param gx[out] 栅格X坐标
    @param gy[out] 栅格Y坐标
    @return bool true=转换成功（在地图范围内），false=超出范围
    @note 以无人机当前位置为中心，覆盖[-5.0,5.0]米范围
    */
    bool world_to_grid(float wx, float wy, int &gx, int &gy) const
    {
        // <第1次修改> 位置：OccupancyGrid2D::world_to_grid函数
        // 思路：动态地图坐标转换（以无人机当前位置为中心）
        gx = static_cast<int>((wx - drone_x) / resolution + 50.0f); // 50为中心栅格(0,0)
        gy = static_cast<int>((wy - drone_y) / resolution + 50.0f);
        return (gx >= 0 && gx < 100 && gy >= 0 && gy < 100);
    }

    /**
    @brief 栅格坐标 → 世界坐标转换（动态地图）
    @param gx 栅格X坐标
    @param gy 栅格Y坐标
    @param wx[out] 世界X坐标（米）
    @param wy[out] 世界Y坐标（米）
    */
    void grid_to_world(int gx, int gy, float &wx, float &wy) const
    {
        // <第1次修改> 位置：OccupancyGrid2D::grid_to_world函数
        // 思路：动态地图坐标反转换
        wx = drone_x + (gx - 50.0f) * resolution;
        wy = drone_y + (gy - 50.0f) * resolution;
    }

    /**
    @brief 更新障碍物地图（含衰减机制 + 动态地图中心）
    @param obstacles 障碍物列表（来自PCL检测）
    @param drone_radius 无人机半径（米）- 用于障碍物膨胀
    @param safety_margin 安全裕度（米）- 保留接口但不用于膨胀
    @param current_drone_x 无人机当前位置X（世界坐标系）
    @param current_drone_y 无人机当前位置Y（世界坐标系）
    @details
    1. 衰减旧障碍物：cells[i][j] -= 2（模拟动态环境）
    2. 障碍物膨胀：圆形膨胀，半径=障碍物半径+无人机半径（无额外安全裕度）
    3. 动态地图中心：以无人机当前位置为中心更新地图
    @note A*规划使用最小安全膨胀，VFH+负责精细避障
    */
    void update_with_obstacles(
        const std::vector<Obstacle> &obstacles,
        float drone_radius,  // 用于障碍物膨胀
        float safety_margin, // 保留接口但不用于膨胀
        float current_drone_x,
        float current_drone_y)
    {
        // <第1次修改> 位置：OccupancyGrid2D::update_with_obstacles函数开头
        // 思路：更新动态地图中心（以无人机当前位置为中心）
        drone_x = current_drone_x;
        drone_y = current_drone_y;

        // 1. 衰减旧障碍物（模拟动态环境）
        for (int i = 0; i < 100; ++i)
        {
            for (int j = 0; j < 100; ++j)
            {
                if (cells[i][j] > 0)
                {
                    cells[i][j] = std::max(static_cast<uint8_t>(0),
                                           static_cast<uint8_t>(cells[i][j] - 2));
                }
            }
        }

        // 2. 障碍物膨胀（圆形，半径=障碍物半径+无人机半径，无额外安全裕度）
        // <第3次修改> 位置：OccupancyGrid2D::update_with_obstacles函数膨胀逻辑
        // 思路：移除0.2m安全裕度，仅使用障碍物半径+无人机半径
        for (const auto &obs : obstacles)
        {
            int gx, gy;
            if (world_to_grid(obs.position.x(), obs.position.y(), gx, gy))
            {
                // 膨胀半径计算：障碍物半径 + 无人机半径（无额外安全裕度）
                float expansion_radius = obs.radius + drone_radius; // 关键修复：移除+0.2f
                int expansion_cells = static_cast<int>(std::ceil(expansion_radius / resolution));

                // 圆形膨胀
                for (int dx = -expansion_cells; dx <= expansion_cells; ++dx)
                {
                    for (int dy = -expansion_cells; dy <= expansion_cells; ++dy)
                    {
                        int nx = gx + dx;
                        int ny = gy + dy;
                        if (nx >= 0 && nx < 100 && ny >= 0 && ny < 100)
                        {
                            float dist = std::sqrt(dx * dx + dy * dy);
                            if (dist <= expansion_cells)
                            {
                                // 距离中心越近，栅格值越高（100→0线性衰减）
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

    /**
    @brief 目标点投影到地图边界（处理超出范围的目标）
    @param start_x 起点X（世界坐标系）
    @param start_y 起点Y（世界坐标系）
    @param goal_x 目标X（世界坐标系）
    @param goal_y 目标Y（世界坐标系）
    @param projected_x[out] 投影后目标X
    @param projected_y[out] 投影后目标Y
    @return bool true=需要投影（目标超出范围），false=无需投影
    @details
    投影逻辑：计算起点→目标连线与地图边界的交点
    地图边界：[drone_x-5.0, drone_x+5.0] × [drone_y-5.0, drone_y+5.0]
    */
    bool project_goal_to_boundary(
        float start_x, float start_y,
        float goal_x, float goal_y,
        float &projected_x, float &projected_y) const
    {
        // <第2次修改> 位置：新增OccupancyGrid2D::project_goal_to_boundary函数
        // 思路：目标点超出地图时投影到边界（连线交点法）
        float min_x = drone_x - 5.0f;
        float max_x = drone_x + 5.0f;
        float min_y = drone_y - 5.0f;
        float max_y = drone_y + 5.0f;

        // 检查目标是否在地图内
        if (goal_x >= min_x && goal_x <= max_x &&
            goal_y >= min_y && goal_y <= max_y)
        {
            return false; // 无需投影
        }

        // 计算起点→目标的参数方程：P = start + t*(goal-start)
        float dx = goal_x - start_x;
        float dy = goal_y - start_y;

        // 检查与四条边界的交点
        float t_min = std::numeric_limits<float>::max();
        float t_candidates[4] = {0};
        int valid_count = 0;

        // 左边界 x = min_x
        if (std::abs(dx) > 1e-6f)
        {
            float t = (min_x - start_x) / dx;
            if (t > 0)
            {
                float y = start_y + t * dy;
                if (y >= min_y && y <= max_y)
                {
                    t_candidates[valid_count++] = t;
                }
            }
        }

        // 右边界 x = max_x
        if (std::abs(dx) > 1e-6f)
        {
            float t = (max_x - start_x) / dx;
            if (t > 0)
            {
                float y = start_y + t * dy;
                if (y >= min_y && y <= max_y)
                {
                    t_candidates[valid_count++] = t;
                }
            }
        }

        // 下边界 y = min_y
        if (std::abs(dy) > 1e-6f)
        {
            float t = (min_y - start_y) / dy;
            if (t > 0)
            {
                float x = start_x + t * dx;
                if (x >= min_x && x <= max_x)
                {
                    t_candidates[valid_count++] = t;
                }
            }
        }

        // 上边界 y = max_y
        if (std::abs(dy) > 1e-6f)
        {
            float t = (max_y - start_y) / dy;
            if (t > 0)
            {
                float x = start_x + t * dx;
                if (x >= min_x && x <= max_x)
                {
                    t_candidates[valid_count++] = t;
                }
            }
        }

        // 选择最近的交点（最小t>0）
        for (int i = 0; i < valid_count; ++i)
        {
            if (t_candidates[i] < t_min)
            {
                t_min = t_candidates[i];
            }
        }

        if (t_min == std::numeric_limits<float>::max())
        {
            // 无有效交点（起点在边界外），使用最近边界点
            projected_x = std::max(min_x, std::min(goal_x, max_x));
            projected_y = std::max(min_y, std::min(goal_y, max_y));
            ROS_WARN("[A*] 无有效投影交点，使用最近边界点(%.2f,%.2f)", projected_x, projected_y);
        }
        else
        {
            projected_x = start_x + t_min * dx;
            projected_y = start_y + t_min * dy;
        }

        ROS_INFO("[A*] 目标点投影: 原(%.2f,%.2f) → 投影(%.2f,%.2f)",
                 goal_x, goal_y, projected_x, projected_y);
        return true;
    }
};
// ============================================================================
// 辅助函数：道格拉斯-普克路径简化
// ============================================================================
/**
@brief 道格拉斯-普克算法简化路径点
@param points 原始路径点列表
@param epsilon 简化阈值（米）
@param simplified 简化后的路径点列表
@details
算法原理：递归寻找距离连线最远的点，若距离>阈值则保留该点并递归处理两侧
@note 阈值0.3米可减少50%~70%路径点，同时保留主要拐点
*/
void douglas_peucker(
    const std::vector<std::pair<float, float>> &points,
    float epsilon,
    std::vector<std::pair<float, float>> &simplified)
{
    // <第4次修改> 位置：新增douglas_peucker函数
    // 思路：路径点简化（道格拉斯-普克算法，阈值0.3米）
    if (points.empty())
        return;

    // 递归辅助函数
    std::function<void(int, int)> dp_recursion = [&](int start_idx, int end_idx)
    {
        if (end_idx - start_idx <= 1)
        {
            if (simplified.empty() || simplified.back() != points[start_idx])
            {
                simplified.push_back(points[start_idx]);
            }
            if (start_idx != end_idx && (simplified.empty() || simplified.back() != points[end_idx]))
            {
                simplified.push_back(points[end_idx]);
            }
            return;
        }

        // 计算起点到终点的直线
        float x1 = points[start_idx].first;
        float y1 = points[start_idx].second;
        float x2 = points[end_idx].first;
        float y2 = points[end_idx].second;
        float line_len = std::sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));

        // 寻找最远点
        int farthest_idx = start_idx;
        float max_dist = 0;

        for (int i = start_idx + 1; i < end_idx; ++i)
        {
            // 点到直线的距离公式
            float dist = std::abs((y2 - y1) * points[i].first - (x2 - x1) * points[i].second + x2 * y1 - y2 * x1) / line_len;
            if (dist > max_dist)
            {
                max_dist = dist;
                farthest_idx = i;
            }
        }

        // 递归条件
        if (max_dist > epsilon)
        {
            dp_recursion(start_idx, farthest_idx);
            dp_recursion(farthest_idx, end_idx);
        }
        else
        {
            if (simplified.empty() || simplified.back() != points[start_idx])
            {
                simplified.push_back(points[start_idx]);
            }
            if (simplified.empty() || simplified.back() != points[end_idx])
            {
                simplified.push_back(points[end_idx]);
            }
        }
    };

    dp_recursion(0, points.size() - 1);
}
// ============================================================================
// 核心算法1：A*全局路径规划（动态地图+目标投影）
// ============================================================================
int astar_plan(
    OccupancyGrid2D &grid, // 非const：需要更新地图中心
    float start_x, float start_y,
    float goal_x, float goal_y,
    float *path_x, float *path_y,
    int max_points);
// ============================================================================
// 核心算法2：增量A*路径规划（重规划使用）
// ============================================================================
int incremental_astar_plan(
    OccupancyGrid2D &grid, // 非const：需要更新地图中心
    float start_x, float start_y,
    float goal_x, float goal_y,
    float *path_x, float *path_y,
    int max_points);
// ============================================================================
// 核心算法3：走廊生成器（B-spline插值 + 曲率动态宽度）
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
// 核心算法4：增强VFH+避障（含走廊软约束 + 异常检测）
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