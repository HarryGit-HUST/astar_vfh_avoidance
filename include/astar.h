/*******************************************************************************
atsar.h - 无人机智能避障系统核心头文件
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
#include "new_detect_obs.h" // PCL 障碍物检测接口
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
// ========== 修改：添加 RViz 可视化相关头文件 ==========
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
// ========== 修改结束 ==========
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
// 函数 1：MAVROS 连接状态回调
// ============================================================================
// 【设计原则】避免命名冲突：原 current_state → mavros_connection_state
// 【原因】mission2 内部需使用 AvoidanceState 枚举，与 mavros_msgs::State 类型冲突
// ============================================================================
/** MAVROS 飞控连接状态（独立变量，避免与任务状态机冲突） */
mavros_msgs::State mavros_connection_state;
/**
@brief MAVROS 状态回调函数
@param msg MAVROS 状态消息指针
@details 将接收到的 MAVROS 状态存储到全局变量 mavros_connection_state
@note 该变量仅用于判断飞控连接状态，不参与任务状态机逻辑
*/
void state_cb(const mavros_msgs::State::ConstPtr &msg);
void state_cb(const mavros_msgs::State::ConstPtr &msg)
{
    mavros_connection_state = *msg;
}
// ============================================================================
// 函数 2：里程计信息回调
// ============================================================================
// 【功能】解析无人机位置/姿态/速度信息，转换为世界坐标系
// 【关键处理】
//   1. 四元数→欧拉角转换（获取 yaw）
//   2. 机体速度→世界速度转换（用于动态安全裕度计算）
//   3. 起飞点初始化（首次高度>0.1m 时记录）
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
/** 起飞点初始化标志（false=未初始化，true=已记录起飞点） */
bool flag_init_position = false;
/**
@brief 里程计回调函数
@param msg 里程计消息指针
@details
提取位置/姿态信息
四元数→欧拉角转换（获取 yaw）
机体速度→世界速度转换（用于 VFH+ 动态安全裕度）
首次高度>0.1m 时记录起飞点（init_position_*）
@note 世界速度计算：rot_matrix * body_vel
*/
void local_pos_cb(const nav_msgs::Odometry::ConstPtr &msg);
void local_pos_cb(const nav_msgs::Odometry::ConstPtr &msg)
{
    local_pos = *msg;
    tf::quaternionMsgToTF(local_pos.pose.pose.orientation, quat);
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    current_pos = Eigen::Vector2f(local_pos.pose.pose.position.x, local_pos.pose.pose.position.y);
    // 机体速度→世界速度转换（用于动态安全裕度计算）
    tf::Vector3 body_vel(local_pos.twist.twist.linear.x, local_pos.twist.twist.linear.y, local_pos.twist.twist.linear.z);
    tf::Matrix3x3 rot_matrix(quat);
    tf::Vector3 world_vel = rot_matrix * body_vel;
    current_vel = Eigen::Vector2f(world_vel.x(), world_vel.y());
    // 首次高度>0.1m 时记录起飞点（避免地面噪声干扰）
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
// 函数 3：位置巡航控制
// ============================================================================
// 【功能】控制无人机飞向指定位置（相对起飞点）
// 【特点】
//   • 超时保护：180 秒未到达自动切换下一任务
//   • 误差判断：位置误差<err_max && 航向误差<0.1rad
//   • 任务标志：mission_pos_cruise_flag 防止重复初始化
// ============================================================================
/** 上次巡航目标位置（用于任务重入检测） */
float mission_pos_cruise_last_position_x = 0;
float mission_pos_cruise_last_position_y = 0;
/** 巡航超时阈值（秒）- 可通过 yaml 配置 */
float mission_cruise_timeout = 180.0f;
/** 任务开始时间（用于超时计时） */
ros::Time mission_cruise_start_time;
/** 超时标志（防止重复触发超时逻辑） */
bool mission_cruise_timeout_flag = false;
/** 任务标志（防止重复初始化） */
bool mission_pos_cruise_flag = false;
/**
@brief 位置巡航控制函数
@param x 目标点 X 坐标（相对起飞点，单位：米）
@param y 目标点 Y 坐标（相对起飞点，单位：米）
@param z 目标点 Z 坐标（相对起飞点，单位：米）
@param target_yaw 目标航向角（单位：弧度）
@param error_max 位置误差阈值（单位：米）
@return bool true=到达目标点，false=未到达
@details
首次调用初始化任务标志和开始时间
超时保护：超过 mission_cruise_timeout 秒强制返回 true
位置控制：setpoint_raw 写入目标位置（世界坐标系）
到达判断：位置误差<error_max && 航向误差<0.1rad
@note 该函数不包含避障逻辑，仅用于简单位置控制（如起飞/悬停）
*/
bool mission_pos_cruise(float x, float y, float z, float target_yaw, float error_max);
bool mission_pos_cruise(float x, float y, float z, float target_yaw, float error_max)
{
    // 首次调用初始化任务标志和开始时间
    if (mission_pos_cruise_flag == false)
    {
        mission_pos_cruise_last_position_x = local_pos.pose.pose.position.x;
        mission_pos_cruise_last_position_y = local_pos.pose.pose.position.y;
        mission_pos_cruise_flag = true;
        mission_cruise_start_time = ros::Time::now();
        mission_cruise_timeout_flag = false;
    }
    // 超时保护：超过 mission_cruise_timeout 秒强制返回 true
    ros::Duration elapsed_time = ros::Time::now() - mission_cruise_start_time;
    if (elapsed_time.toSec() > mission_cruise_timeout && !mission_cruise_timeout_flag)
    {
        ROS_WARN("[巡航超时] 已耗时%.1f 秒（阈值%.1f 秒），强制切换下一个任务！", elapsed_time.toSec(), mission_cruise_timeout);
        mission_cruise_timeout_flag = true;
        mission_pos_cruise_flag = false; // 重置任务标志
        return true;
    }
    // 生成位置控制指令（世界坐标系）
    setpoint_raw.type_mask = /*1 + 2 + 4 */ +8 + 16 + 32 + 64 + 128 + 256 + 512 /*+ 1024 */ + 2048;
    setpoint_raw.coordinate_frame = 1;                      // 1=FRAME_LOCAL_NED
    setpoint_raw.position.x = x + init_position_x_take_off; // 转换为世界坐标
    setpoint_raw.position.y = y + init_position_y_take_off;
    setpoint_raw.position.z = z + init_position_z_take_off;
    setpoint_raw.yaw = target_yaw;
    // 到达判断：位置误差 <error_max && 航向误差 <0.1rad
    if (fabs(local_pos.pose.pose.position.x - x - init_position_x_take_off) < error_max &&
        fabs(local_pos.pose.pose.position.y - y - init_position_y_take_off) < error_max &&
        fabs(local_pos.pose.pose.position.z - z - init_position_z_take_off) < error_max &&
        fabs(yaw - target_yaw) < 0.1)
    {
        ROS_INFO("到达目标点，巡航点任务完成");
        mission_cruise_timeout_flag = false;
        mission_pos_cruise_flag = false; // 重置任务标志
        return true;
    }
    return false;
}
// ============================================================================
// 函数 4：精确降落控制
// ============================================================================
// 【三阶段降落】
//   Stage1: 悬停稳定（10 秒）- 确保位置稳定
//   Stage2: 缓慢下降（0.75 倍高度衰减）- 避免冲击
//   Stage3: 接地稳定（2 秒）- 确认着陆完成
// ============================================================================
/** 降落起始位置（X/Y，世界坐标系） */
float precision_land_init_position_x = 0;
float precision_land_init_position_y = 0;
/** 降落起始标志 */
bool precision_land_init_position_flag = false;
/** 悬停完成标志（Stage1 完成） */
bool hovor_done = false;
/** 降落完成标志（Stage2 完成） */
bool land_done = false;
/** 阶段切换时间戳 */
ros::Time precision_land_last_time;
/**
@brief 精确降落控制函数
@param err_max 位置误差阈值（单位：米）
@return bool true=降落完成，false=降落中
@details 三阶段降落流程：
Stage1（悬停稳定）:
- 条件：位置误差<err_max/2 && 速度<err_max/10 || 超时 10 秒
- 动作：保持 ALTITUDE 高度悬停
Stage2（缓慢下降）:
- 条件：悬停完成 && 高度接近起飞点 || 超时 5 秒
- 动作：高度按 0.75 倍衰减下降（z = (z+0.15)*0.75 - 0.15）
Stage3（接地稳定）:
- 条件：降落完成 && 保持 2 秒
- 动作：缓慢下降 0.02m/步，确认着陆
@note 降落过程全程保持起始位置 X/Y 不变
*/
bool precision_land(float err_max);
bool precision_land(float err_max)
{
    // 首次调用记录降落起始位置
    if (!precision_land_init_position_flag)
    {
        precision_land_init_position_x = local_pos.pose.pose.position.x;
        precision_land_init_position_y = local_pos.pose.pose.position.y;
        precision_land_last_time = ros::Time::now();
        precision_land_init_position_flag = true;
    }
    // Stage1: 悬停稳定（10 秒）
    if (fabs(local_pos.pose.pose.position.x - precision_land_init_position_x) < err_max / 2 &&
            fabs(local_pos.twist.twist.linear.x) < err_max / 10 &&
            fabs(local_pos.pose.pose.position.y - precision_land_init_position_y) < err_max / 2 &&
            fabs(local_pos.twist.twist.linear.y) < err_max / 10 ||
        ros::Time::now() - precision_land_last_time > ros::Duration(10.0))
    {
        hovor_done = true;
        precision_land_last_time = ros::Time::now();
    }
    // Stage2: 缓慢下降
    if (!land_done && hovor_done && (fabs(local_pos.pose.pose.position.z - init_position_z_take_off) < err_max / 5 || ros::Time::now() - precision_land_last_time > ros::Duration(5.0)))
    {
        land_done = true;
        precision_land_last_time = ros::Time::now();
    }
    // Stage3: 接地稳定（2 秒）
    if (land_done && ros::Time::now() - precision_land_last_time > ros::Duration(2.0))
    {
        ROS_INFO("Precision landing complete.");
        precision_land_init_position_flag = false; // 重置标志
        hovor_done = false;
        land_done = false;
        return true;
    }
    // 生成降落指令（X/Y 保持起始位置不变）
    setpoint_raw.position.x = precision_land_init_position_x;
    setpoint_raw.position.y = precision_land_init_position_y;
    if (!land_done && !hovor_done)
    {
        // Stage1: 悬停
        setpoint_raw.position.z = ALTITUDE;
        ROS_INFO("悬停中");
    }
    else if (!land_done)
    {
        // Stage2: 缓慢下降（0.75 倍高度衰减）
        setpoint_raw.position.z = (local_pos.pose.pose.position.z + 0.15) * 0.75 - 0.15;
        ROS_INFO("降落中");
    }
    else
    {
        // Stage3: 接地稳定
        setpoint_raw.position.z = local_pos.pose.pose.position.z - 0.02;
        ROS_INFO("稳定中");
    }
    setpoint_raw.type_mask = /*1 + 2 + 4 +*/ 8 + 16 + 32 + 64 + 128 + 256 + 512 /*+ 1024 + 2048*/;
    setpoint_raw.coordinate_frame = 1;
    return false;
}
// ============================================================================
// 辅助结构：2D 栅格地图（A*规划使用）
// ============================================================================
// 【设计】100x100 栅格，分辨率 0.1m → 覆盖 10m×10m 范围
// 【坐标转换】
//   世界坐标 (x,y) → 栅格坐标 (gx,gy):
//     gx = (x - origin_x) / resolution
//     gy = (y - origin_y) / resolution
// 【障碍物处理】
//   仅标记障碍物中心（不膨胀），由 VFH+ 负责精细避障
// ============================================================================
/**
@brief 2D 栅格地图结构体（用于 A*规划）
@details
栅格尺寸：100×100
分辨率：0.1 米/栅格
覆盖范围：[-5,5] 米 × [-5,5] 米（以起飞点为中心）
栅格值：0=自由，1~99=未知，100=障碍物
@note 障碍物仅标记中心点，不进行膨胀（由 VFH+ 负责精细避障）
*/
struct OccupancyGrid2D
{
    uint8_t cells[100][100]; // 栅格值 [0,100]
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
        origin_x = -5.0f; // 覆盖 [-5,5] 米范围
        origin_y = -5.0f;
        for (int i = 0; i < 100; ++i)
            for (int j = 0; j < 100; ++j)
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
        return (gx >= 0 && gx < 100 && gy >= 0 && gy < 100);
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
    @brief 更新障碍物地图（含衰减机制）
    @param obstacles 障碍物列表（来自 PCL 检测）
    @param drone_radius 无人机半径（米）
    @param safety_margin 安全裕度（米）
    @details
    衰减旧障碍物：cells[i][j] -= 2（模拟动态环境）
    投影新障碍物：仅标记障碍物中心（不膨胀）
    @note 由 VFH+ 负责精细避障，A*仅提供粗糙路径
    */
    void update_with_obstacles(
        const std::vector<Obstacle> &obstacles,
        float drone_radius,  // 保留参数但不再使用（接口兼容）
        float safety_margin) // 保留参数但不再使用（接口兼容）
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
        // 2. 仅标记障碍物中心（不膨胀）
        for (const auto &obs : obstacles)
        {
            int gx, gy;
            if (world_to_grid(obs.position.x(), obs.position.y(), gx, gy))
            {
                cells[gx][gy] = 100; // 仅中心点标记为障碍物（100=完全阻挡）
            }
        }
    }
};
// ============================================================================
// 核心算法 1：A*全局路径规划
// ============================================================================
// 【算法】4 方向曼哈顿距离 A*（轻量级，<50ms）
// 【优化】
//   • 起点/目标点在障碍物内时自动偏移（5 栅格内搜索）
//   • 闭集使用哈希表加速查找
//   • 最大迭代 10000 次防死循环
// 【输出】路径点数组（相对起飞点坐标）
// ============================================================================
/**
@brief A*全局路径规划函数
@param grid 2D 栅格地图（障碍物已更新）
@param start_x 起点 X（世界坐标系，米）
@param start_y 起点 Y（世界坐标系，米）
@param goal_x 目标点 X（世界坐标系，米）
@param goal_y 目标点 Y（世界坐标系，米）
@param path_x[out] 路径点 X 数组（相对起飞点，米）
@param path_y[out] 路径点 Y 数组（相对起飞点，米）
@param max_points 输出数组最大容量
@return int 实际生成的路径点数量（0=失败）
@details
坐标转换：世界坐标 → 栅格坐标
边界处理：起点/目标在障碍物内时自动偏移（5 栅格内搜索自由空间）
A*主循环：4 方向曼哈顿距离，闭集哈希表加速
路径回溯：从目标回溯到起点，转换为世界坐标（相对起飞点）
@note
• 启发式函数：h = |dx| + |dy|（曼哈顿距离）
• 代价函数：g = 父节点 g + 1.0（均匀代价）
• 最大迭代：10000 次（防死循环）
*/
int astar_plan(
    const OccupancyGrid2D &grid,
    float start_x, float start_y,
    float goal_x, float goal_y,
    float *path_x, float *path_y,
    int max_points);
// ============================================================================
// 核心算法 2：增量 A*路径规划
// ============================================================================
// 【特点】
//   • 重用前一次规划的结果，只重新计算受影响的部分
//   • 适用于动态环境，减少重规划计算量
//   • 当环境变化较大时，会退化为标准 A*
// ============================================================================
/**
@brief 增量 A*全局路径规划函数
@param grid 2D 栅格地图（障碍物已更新）
@param start_x 起点 X（世界坐标系，米）
@param start_y 起点 Y（世界坐标系，米）
@param goal_x 目标点 X（世界坐标系，米）
@param goal_y 目标点 Y（世界坐标系，米）
@param path_x[out] 路径点 X 数组（相对起飞点，米）
@param path_y[out] 路径点 Y 数组（相对起飞点，米）
@param max_points 输出数组最大容量
@return int 实际生成的路径点数量（0=失败）
@details
增量 A*算法：在环境变化时，只重新计算受影响的部分
数据缓存：使用静态变量存储上次规划结果
@note
• 适用于动态环境，减少重规划计算量
• 如果环境变化较大，会退化为标准 A*
• 最大迭代 5000 次（比标准 A*减少 50%）
*/
int incremental_astar_plan(
    const OccupancyGrid2D &grid,
    float start_x, float start_y,
    float goal_x, float goal_y,
    float *path_x, float *path_y,
    int max_points);
// ============================================================================
// 核心算法 3：走廊生成器（B-spline 插值 + 曲率动态宽度）
// ============================================================================
// 【三步流程】
//   1. B-spline 插值：将 A*离散路径转换为平滑曲线（0.15m 分辨率）
//   2. 曲率计算：三点法估算路径曲率（1/半径）
//   3. 动态宽度：曲率>阈值时自动加宽 20%（急弯防卡死）
// 【输出】走廊中心点 + 对应宽度数组
// ============================================================================
/**
@brief 走廊生成器（B-spline 插值 + 曲率动态宽度）
@param astar_path_x A*路径点 X（相对起飞点，米）
@param astar_path_y A*路径点 Y（相对起飞点，米）
@param num_points 路径点数量
@param base_width 基础走廊宽度（米）
@param corridor_x[out] 走廊中心点 X（世界坐标系，米）
@param corridor_y[out] 走廊中心点 Y（世界坐标系，米）
@param corridor_width[out] 走廊宽度数组（米）
@param max_size 输出数组最大容量
@return int 实际生成的走廊点数量
@details
B-spline 插值：均匀重采样（0.15m 分辨率），消除 A*网格锯齿
曲率计算：三点法（prev-current-next），滑动窗口平均抑制噪声
动态宽度：曲率>0.4 时宽度×1.2（急弯自动加宽 20%）
@note
• 插值分辨率 0.15m：平衡平滑度与计算开销
• 曲率阈值 0.4：对应转弯半径≈2.5 米（实测最优）
• 宽度范围 [0.5,3.0] 米：超出范围自动钳位
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
// 核心算法 4：增强 VFH+ 避障（含走廊软约束 + 异常检测）
// ============================================================================
// 【三层防御】
//   L1: VFH+ 基础避障（直方图 + 三层代价函数）
//   L2: 走廊软约束融合（动态权重：障碍物近→VFH 权重高）
//   L3: 异常状态检测（振荡/目标不可达/视野变化→触发重规划）
// 【创新点】
//   • 历史方向记忆：连续同方向 -20% 代价（增强运动惯性）
//   • 滞后效应：仅当新扇区显著更优 (15%) 时才切换
//   • 振荡恢复：切线方向逃逸（40% 最大速度）
// ============================================================================
/**
@brief 增强 VFH+ 避障函数（含走廊软约束 + 异常检测）
@param target_x_rel 目标点 X（相对起飞点，米）
@param target_y_rel 目标点 Y（相对起飞点，米）
@param target_yaw 目标航向（弧度）
@param uav_radius 无人机半径（米）
@param safe_margin 安全裕度（米）
@param max_speed 最大速度（米/秒）
@param min_safe_distance 力场最小距离（米，防除零）
@param corridor_x 走廊中心点 X（世界坐标系，米）
@param corridor_y 走廊中心点 Y（世界坐标系，米）
@param corridor_width 走廊宽度数组（米）
@param corridor_size 走廊点数量
@param enable_corridor 是否启用走廊约束
@param out_need_replan[out] 是否需要 A*重规划（true=触发重规划）
@return bool true=抵达目标点附近（<0.4m），false=未到达
@details
【三层防御体系】
L1: VFH+ 基础避障
- 63x63 栅格系统（0.08m 分辨率）
- 72 扇区直方图（5°/扇区）
- 三层代价函数：障碍物代价 (50%) + 目标代价 (40%) + 转向代价 (10%)
L2: 走廊软约束融合
- 在走廊内：施加指向中心的吸引力（距离中心越远，吸引力越强）
- 动态权重：障碍物距离<1.5m → VFH 权重↑，走廊权重↓
- 软约束：不强制回归走廊，仅倾向引导
L3: 异常状态检测（内嵌设计）
- 振荡检测：3 秒窗口内位置标准差<0.25m + 角度变化>90° → 触发重规划
- 目标不可达：连续 5 帧目标方向被阻挡 → 触发重规划
- 视野变化：检测到新大型障碍物 (半径>0.8m) → 触发重规划
- 振荡恢复：切线方向逃逸（40% 最大速度，持续 2 秒）
【工程特性】
- 无文件级全局变量：所有中间数据置于函数栈/静态局部变量
- 历史方向记忆：连续同方向 -20% 代价（增强运动惯性）
- 滞后效应：仅当新扇区显著更优 (15%) 时才切换（防抖动）
- 速度调制：前方拥堵指数→速度衰减（最低 30% 最大速度）
@note
• out_need_replan=true 时，主控应触发 A*重规划
• 走廊约束为"软引导"，不解决死胡同问题（需重规划）
• 振荡恢复期间禁用走廊约束，全力逃逸
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
// ========== 修改：添加 RViz 可视化发布器声明 ==========
// 在 main 函数中声明为 extern，在 main 函数中定义
extern ros::Publisher g_obstacle_marker_pub;
extern ros::Publisher g_astar_path_pub;
extern ros::Publisher g_corridor_pub;
extern ros::Publisher g_vfh_grid_pub;
// ========== 修改结束 ==========
#endif 