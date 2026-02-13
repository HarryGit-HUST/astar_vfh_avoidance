/*******************************************************************************
astar.h - 无人机智能避障系统核心头文件
============================================================================
系统架构：三层防御体系
L1: A*全局规划（战略层） - 生成初始路径（带障碍物膨胀）
L2: 走廊生成器（战术层） - 将离散路径转换为平滑走廊（B-spline + 曲率自适应）
L3: VFH+避障（执行层） - 局部避障 + 走廊软约束融合 + 异常检测
异常处理内嵌设计：
• 振荡检测：位置标准差<0.25m + 角度变化>90° → 触发重规划
• 路径阻塞检测：规划路径被新障碍物占据 → 触发重规划
• 视野变化：检测到新大型障碍物(半径>0.8m) → 触发重规划
• 振荡恢复：切线方向逃逸策略（40%最大速度）
工程规范：
• 首次规划：标准A*（全局搜索，带障碍物膨胀）
• 重规划：增量A*（仅重算受影响区域）
• 障碍物膨胀：A*使用障碍物半径+无人机半径+0.2m安全裕度
• 无文件级全局变量：所有中间数据置于函数栈/静态局部变量
坐标系约定：
• 世界坐标系：Gazebo全局坐标（原点=起飞点）
• 相对坐标系：任务目标点（相对起飞点，单位：米）
• 机体坐标系：无人机前向为X+，右向为Y+
版本：2.1 (2026-02-14)
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
栅格尺寸：100×100
分辨率：0.1米/栅格
覆盖范围：[-5,5]米 × [-5,5]米（以起飞点为中心）
栅格值：0=自由, 1~99=未知, 100=障碍物
@note 障碍物膨胀采用圆形膨胀，膨胀半径=障碍物半径+无人机半径+0.2m安全裕度
*/
struct OccupancyGrid2D
{
    uint8_t cells[100][100]; // 栅格值 [0,100]
    float resolution;        // 栅格分辨率（米）
    float origin_x;          // 地图原点X（世界坐标系）
    float origin_y;          // 地图原点Y（世界坐标系）

    OccupancyGrid2D()
    {
        resolution = 0.1f;
        origin_x = -5.0f;
        origin_y = -5.0f;
        for (int i = 0; i < 100; ++i)
            for (int j = 0; j < 100; ++j)
                cells[i][j] = 0;
    }

    bool world_to_grid(float wx, float wy, int &gx, int &gy) const
    {
        gx = static_cast<int>((wx - origin_x) / resolution);
        gy = static_cast<int>((wy - origin_y) / resolution);
        return (gx >= 0 && gx < 100 && gy >= 0 && gy < 100);
    }

    void grid_to_world(int gx, int gy, float &wx, float &wy) const
    {
        wx = origin_x + gx * resolution;
        wy = origin_y + gy * resolution;
    }

    /**
    @brief 更新障碍物地图（含衰减机制 + 障碍物膨胀）
    @param obstacles 障碍物列表（来自PCL检测）
    @param drone_radius 无人机半径（米）- 用于障碍物膨胀
    @param safety_margin 安全裕度（米）- 保留接口但不用于膨胀（A*使用固定0.2m裕度）
    @details
    1. 衰减旧障碍物：cells[i][j] -= 2（模拟动态环境）
    2. 障碍物膨胀：圆形膨胀，半径=障碍物半径+无人机半径+0.2m
    @note A*规划使用固定0.2m安全裕度，确保路径远离障碍物
    */
    void update_with_obstacles(
        const std::vector<Obstacle> &obstacles,
        float drone_radius,  // 用于障碍物膨胀
        float safety_margin) // 保留接口但不用于膨胀
    {
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

        // 2. 障碍物膨胀（圆形，半径=障碍物半径+无人机半径+0.2m）
        for (const auto &obs : obstacles)
        {
            int gx, gy;
            if (world_to_grid(obs.position.x(), obs.position.y(), gx, gy))
            {
                // 膨胀半径计算：障碍物半径 + 无人机半径 + 0.2m安全裕度
                float expansion_radius = obs.radius + drone_radius + 0.2f;
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
};
// ============================================================================
// 核心算法1：A*全局路径规划（首次规划使用）
// ============================================================================
int astar_plan(
    const OccupancyGrid2D &grid,
    float start_x, float start_y,
    float goal_x, float goal_y,
    float *path_x, float *path_y,
    int max_points);
// ============================================================================
// 核心算法2：增量A*路径规划（重规划使用）
// ============================================================================
int incremental_astar_plan(
    const OccupancyGrid2D &grid,
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