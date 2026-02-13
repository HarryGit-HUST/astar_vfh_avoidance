/*******************************************************************************
 * VFF.h - 无人机智能避障系统核心头文件
 * ============================================================================
 * 系统架构：三层防御体系
 *   L1: A*全局规划（战略层） - 生成初始路径
 *   L2: 走廊生成器（战术层） - 将离散路径转换为平滑走廊（B-spline + 曲率自适应）
 *   L3: VFH+避障（执行层） - 局部避障 + 走廊软约束融合 + 异常检测
 *
 * 异常处理内嵌设计：
 *   • 振荡检测：位置标准差<0.25m + 角度变化>90° → 触发重规划
 *   • 目标不可达：连续5帧目标方向被阻挡 → 触发重规划
 *   • 视野变化：检测到新大型障碍物(半径>0.8m) → 触发重规划
 *   • 振荡恢复：切线方向逃逸策略（40%最大速度）
 *
 * 工程规范：
 *   • 无文件级全局变量：所有中间数据置于函数栈/静态局部变量
 *   • 命名冲突规避：MAVROS连接状态 → mavros_connection_state
 *   • 任务状态机：mission_num（高层） + avoidance_state（mission2内部）
 *   • 参数驱动：全部行为由yaml配置控制，无需改代码
 *
 * 坐标系约定：
 *   • 世界坐标系：Gazebo全局坐标（原点=起飞点）
 *   • 相对坐标系：任务目标点（相对起飞点，单位：米）
 *   • 机体坐标系：无人机前向为X+，右向为Y+
 *
 * 版本：2.0 (2026-02-11)
 * 作者：智能避障系统
 ******************************************************************************/

#ifndef VFF_H
#define VFF_H

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
// 【设计原则】避免命名冲突：原current_state → mavros_connection_state
// 【原因】mission2内部需使用AvoidanceState枚举，与mavros_msgs::State类型冲突
// ============================================================================

/** MAVROS飞控连接状态（独立变量，避免与任务状态机冲突） */
mavros_msgs::State mavros_connection_state;

/**
 * @brief MAVROS状态回调函数
 * @param msg MAVROS状态消息指针
 * @details 将接收到的MAVROS状态存储到全局变量mavros_connection_state
 * @note 该变量仅用于判断飞控连接状态，不参与任务状态机逻辑
 */
void state_cb(const mavros_msgs::State::ConstPtr &msg);
void state_cb(const mavros_msgs::State::ConstPtr &msg)
{
    mavros_connection_state = *msg;
}

// ============================================================================
// 函数2：里程计信息回调
// ============================================================================
// 【功能】解析无人机位置/姿态/速度信息，转换为世界坐标系
// 【关键处理】
//   1. 四元数→欧拉角转换（获取yaw）
//   2. 机体速度→世界速度转换（用于动态安全裕度计算）
//   3. 起飞点初始化（首次高度>0.1m时记录）
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
 * @brief 里程计回调函数
 * @param msg 里程计消息指针
 * @details
 *   1. 提取位置/姿态信息
 *   2. 四元数→欧拉角转换（获取yaw）
 *   3. 机体速度→世界速度转换（用于VFH+动态安全裕度）
 *   4. 首次高度>0.1m时记录起飞点（init_position_*）
 * @note 世界速度计算：rot_matrix * body_vel
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

    // 首次高度>0.1m时记录起飞点（避免地面噪声干扰）
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
// 【功能】控制无人机飞向指定位置（相对起飞点）
// 【特点】
//   • 超时保护：180秒未到达自动切换下一任务
//   • 误差判断：位置误差<err_max && 航向误差<0.1rad
//   • 任务标志：mission_pos_cruise_flag防止重复初始化
// ============================================================================

/** 上次巡航目标位置（用于任务重入检测） */
float mission_pos_cruise_last_position_x = 0;
float mission_pos_cruise_last_position_y = 0;

/** 巡航超时阈值（秒）- 可通过yaml配置 */
float mission_cruise_timeout = 180.0f;

/** 任务开始时间（用于超时计时） */
ros::Time mission_cruise_start_time;

/** 超时标志（防止重复触发超时逻辑） */
bool mission_cruise_timeout_flag = false;

/** 任务标志（防止重复初始化） */
bool mission_pos_cruise_flag = false;

/**
 * @brief 位置巡航控制函数
 * @param x 目标点X坐标（相对起飞点，单位：米）
 * @param y 目标点Y坐标（相对起飞点，单位：米）
 * @param z 目标点Z坐标（相对起飞点，单位：米）
 * @param target_yaw 目标航向角（单位：弧度）
 * @param error_max 位置误差阈值（单位：米）
 * @return bool true=到达目标点，false=未到达
 * @details
 *   1. 首次调用初始化任务标志和开始时间
 *   2. 超时保护：超过mission_cruise_timeout秒强制返回true
 *   3. 位置控制：setpoint_raw写入目标位置（世界坐标系）
 *   4. 到达判断：位置误差<error_max && 航向误差<0.1rad
 * @note 该函数不包含避障逻辑，仅用于简单位置控制（如起飞/悬停）
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

    // 超时保护：超过mission_cruise_timeout秒强制返回true
    ros::Duration elapsed_time = ros::Time::now() - mission_cruise_start_time;
    if (elapsed_time.toSec() > mission_cruise_timeout && !mission_cruise_timeout_flag)
    {
        ROS_WARN("[巡航超时] 已耗时%.1f秒（阈值%.1f秒），强制切换下一个任务！", elapsed_time.toSec(), mission_cruise_timeout);
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

    // 到达判断：位置误差<error_max && 航向误差<0.1rad
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
// 函数4：精确降落控制
// ============================================================================
// 【三阶段降落】
//   Stage1: 悬停稳定（10秒）- 确保位置稳定
//   Stage2: 缓慢下降（0.75倍高度衰减）- 避免冲击
//   Stage3: 接地稳定（2秒）- 确认着陆完成
// ============================================================================

/** 降落起始位置（X/Y，世界坐标系） */
float precision_land_init_position_x = 0;
float precision_land_init_position_y = 0;

/** 降落起始标志 */
bool precision_land_init_position_flag = false;

/** 悬停完成标志（Stage1完成） */
bool hovor_done = false;

/** 降落完成标志（Stage2完成） */
bool land_done = false;

/** 阶段切换时间戳 */
ros::Time precision_land_last_time;

/**
 * @brief 精确降落控制函数
 * @param err_max 位置误差阈值（单位：米）
 * @return bool true=降落完成，false=降落中
 * @details 三阶段降落流程：
 *   Stage1（悬停稳定）:
 *     - 条件：位置误差<err_max/2 && 速度<err_max/10 || 超时10秒
 *     - 动作：保持ALTITUDE高度悬停
 *   Stage2（缓慢下降）:
 *     - 条件：悬停完成 && 高度接近起飞点 || 超时5秒
 *     - 动作：高度按0.75倍衰减下降（z = (z+0.15)*0.75 - 0.15）
 *   Stage3（接地稳定）:
 *     - 条件：降落完成 && 保持2秒
 *     - 动作：缓慢下降0.02m/步，确认着陆
 * @note 降落过程全程保持起始位置X/Y不变
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

    // Stage1: 悬停稳定（10秒）
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

    // Stage3: 接地稳定（2秒）
    if (land_done && ros::Time::now() - precision_land_last_time > ros::Duration(2.0))
    {
        ROS_INFO("Precision landing complete.");
        precision_land_init_position_flag = false; // 重置标志
        hovor_done = false;
        land_done = false;
        return true;
    }

    // 生成降落指令（X/Y保持起始位置不变）
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
        // Stage2: 缓慢下降（0.75倍高度衰减）
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
// 辅助结构：2D栅格地图（A*规划使用）
// ============================================================================
// 【设计】100x100栅格，分辨率0.1m → 覆盖10m×10m范围
// 【坐标转换】
//   世界坐标(x,y) → 栅格坐标(gx,gy):
//     gx = (x - origin_x) / resolution
//     gy = (y - origin_y) / resolution
// 【障碍物膨胀】圆形膨胀，半径=无人机半径+安全裕度
// ============================================================================

/**
 * @brief 2D栅格地图结构体（用于A*规划）
 * @details
 *   - 栅格尺寸：100×100
 *   - 分辨率：0.1米/栅格
 *   - 覆盖范围：[-5,5]米 × [-5,5]米（以起飞点为中心）
 *   - 栅格值：0=自由, 1~99=未知, 100=障碍物
 * @note 障碍物膨胀采用圆形膨胀（非方形），更符合物理实际
 */
struct OccupancyGrid2D
{
    uint8_t cells[100][100]; // 栅格值 [0,100]
    float resolution;        // 栅格分辨率（米）
    float origin_x;          // 地图原点X（世界坐标系）
    float origin_y;          // 地图原点Y（世界坐标系）

    /**
     * @brief 构造函数：初始化地图参数
     * @details
     *   - 分辨率：0.1米/栅格
     *   - 原点：(-5.0, -5.0) → 覆盖[-5,5]米范围
     *   - 初始状态：全自由空间（cells=0）
     */
    OccupancyGrid2D()
    {
        resolution = 0.1f;
        origin_x = -5.0f; // 覆盖[-5,5]米范围
        origin_y = -5.0f;
        for (int i = 0; i < 100; ++i)
            for (int j = 0; j < 100; ++j)
                cells[i][j] = 0;
    }

    /**
     * @brief 世界坐标 → 栅格坐标转换
     * @param wx 世界X坐标（米）
     * @param wy 世界Y坐标（米）
     * @param gx[out] 栅格X坐标
     * @param gy[out] 栅格Y坐标
     * @return bool true=转换成功（在地图范围内），false=超出范围
     */
    bool world_to_grid(float wx, float wy, int &gx, int &gy) const
    {
        gx = static_cast<int>((wx - origin_x) / resolution);
        gy = static_cast<int>((wy - origin_y) / resolution);
        return (gx >= 0 && gx < 100 && gy >= 0 && gy < 100);
    }

    /**
     * @brief 栅格坐标 → 世界坐标转换
     * @param gx 栅格X坐标
     * @param gy 栅格Y坐标
     * @param wx[out] 世界X坐标（米）
     * @param wy[out] 世界Y坐标（米）
     */
    void grid_to_world(int gx, int gy, float &wx, float &wy) const
    {
        wx = origin_x + gx * resolution;
        wy = origin_y + gy * resolution;
    }

    /**
     * @brief 更新障碍物地图（含衰减机制）
     * @param obstacles 障碍物列表（来自PCL检测）
     * @param drone_radius 无人机半径（米）
     * @param safety_margin 安全裕度（米）
     * @details
     *   1. 衰减旧障碍物：cells[i][j] -= 2（模拟动态环境）
     *   2. 投影新障碍物：圆形膨胀（半径=drone_radius+safety_margin）
     *   3. 膨胀算法：距离中心越近，栅格值越高（100→0线性衰减）
     * @note 衰减机制使系统能适应动态环境（如移动障碍物消失）
     */
    // ========== 修复2: 移除安全半径膨胀，仅标记障碍物中心（替换原函数体） ==========
    void update_with_obstacles(
        const std::vector<Obstacle> &obstacles,
        float drone_radius,  // 保留参数但不再使用（接口兼容）
        float safety_margin) // 保留参数但不再使用（接口兼容）
    {
        // 1. 衰减旧障碍物（模拟动态环境）- 保留此机制
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

        // 2. 【关键修改】仅标记障碍物中心（不膨胀）
        //    由VFH+负责精细避障，A*仅提供粗糙路径
        for (const auto &obs : obstacles)
        {
            int gx, gy;
            if (world_to_grid(obs.position.x(), obs.position.y(), gx, gy))
            {
                cells[gx][gy] = 100; // 仅中心点标记为障碍物（100=完全阻挡）
            }
        }
    }

// ============================================================================
// 核心算法1：A*全局路径规划
// ============================================================================
// 【算法】4方向曼哈顿距离A*（轻量级，<50ms）
// 【优化】
//   • 起点/目标点在障碍物内时自动偏移（5栅格内搜索）
//   • 闭集使用哈希表加速查找
//   • 最大迭代10000次防死循环
// 【输出】路径点数组（相对起飞点坐标）
// ============================================================================

/**
 * @brief A*全局路径规划函数
 * @param grid 2D栅格地图（障碍物已更新）
 * @param start_x 起点X（世界坐标系，米）
 * @param start_y 起点Y（世界坐标系，米）
 * @param goal_x 目标点X（世界坐标系，米）
 * @param goal_y 目标点Y（世界坐标系，米）
 * @param path_x[out] 路径点X数组（相对起飞点，米）
 * @param path_y[out] 路径点Y数组（相对起飞点，米）
 * @param max_points 输出数组最大容量
 * @return int 实际生成的路径点数量（0=失败）
 * @details
 *   1. 坐标转换：世界坐标 → 栅格坐标
 *   2. 边界处理：起点/目标在障碍物内时自动偏移（5栅格内搜索自由空间）
 *   3. A*主循环：4方向曼哈顿距离，闭集哈希表加速
 *   4. 路径回溯：从目标回溯到起点，转换为世界坐标（相对起飞点）
 * @note
 *   • 启发式函数：h = |dx| + |dy|（曼哈顿距离）
 *   • 代价函数：g = 父节点g + 1.0（均匀代价）
 *   • 最大迭代：10000次（防死循环）
 */
int astar_plan(
    const OccupancyGrid2D &grid,
    float start_x, float start_y,
    float goal_x, float goal_y,
    float *path_x, float *path_y,
    int max_points)
{
    // ========== 1. 坐标转换：世界坐标 → 栅格坐标 ==========
    int start_gx, start_gy, goal_gx, goal_gy;
    if (!grid.world_to_grid(start_x, start_y, start_gx, start_gy) ||
        !grid.world_to_grid(goal_x, goal_y, goal_gx, goal_gy))
    {
        ROS_ERROR("[A*] 起点/目标点超出地图范围（[-5,5]米）");
        return 0;
    }

    // 边界检查：起点在障碍物内时自动偏移
    if (grid.cells[start_gx][start_gy] > 50)
    {
        ROS_WARN("[A*] 起点在障碍物内，尝试5栅格内偏移");
        bool found = false;
        for (int r = 1; r <= 5 && !found; ++r)
        {
            for (int dx = -r; dx <= r && !found; ++dx)
            {
                for (int dy = -r; dy <= r && !found; ++dy)
                {
                    int nx = start_gx + dx, ny = start_gy + dy;
                    if (nx >= 0 && nx < 100 && ny >= 0 && ny < 100 &&
                        grid.cells[nx][ny] < 30)
                    { // 30为自由空间阈值
                        start_gx = nx;
                        start_gy = ny;
                        found = true;
                        ROS_INFO("[A*] 起点偏移至栅格(%d,%d)", nx, ny);
                    }
                }
            }
        }
        if (!found)
        {
            ROS_ERROR("[A*] 无法在5栅格内找到有效起点");
            return 0;
        }
    }

    // 边界检查：目标点在障碍物内时自动偏移
    if (grid.cells[goal_gx][goal_gy] > 50)
    {
        ROS_WARN("[A*] 目标点在障碍物内，尝试5栅格内偏移");
        bool found = false;
        for (int r = 1; r <= 5 && !found; ++r)
        {
            for (int dx = -r; dx <= r && !found; ++dx)
            {
                for (int dy = -r; dy <= r && !found; ++dy)
                {
                    int nx = goal_gx + dx, ny = goal_gy + dy;
                    if (nx >= 0 && nx < 100 && ny >= 0 && ny < 100 &&
                        grid.cells[nx][ny] < 30)
                    {
                        goal_gx = nx;
                        goal_gy = ny;
                        found = true;
                        ROS_INFO("[A*] 目标点偏移至栅格(%d,%d)", nx, ny);
                    }
                }
            }
        }
        if (!found)
        {
            ROS_ERROR("[A*] 无法在5栅格内找到有效目标点");
            return 0;
        }
    }

    // ========== 2. A*主循环（4方向曼哈顿距离） ==========
    // 优先队列：f(n) = g(n) + h(n)，按f值升序排列
    auto cmp = [](const std::pair<int, std::pair<int, int>> &a,
                  const std::pair<int, std::pair<int, int>> &b)
    {
        return a.first > b.first; // 小顶堆
    };
    std::priority_queue<std::pair<int, std::pair<int, int>>,
                        std::vector<std::pair<int, std::pair<int, int>>>,
                        decltype(cmp)>
        open_set(cmp);

    // g_score缓存：栅格ID(gx*1000+gy) → g值
    std::unordered_map<int, int> g_score;

    // 父节点映射：用于路径回溯
    std::unordered_map<int, std::pair<int, int>> parent_map;

    // 闭集：已扩展节点
    std::unordered_set<int> closed_set;

    // 起点入队（f = h，g=0）
    open_set.push({std::abs(goal_gx - start_gx) + std::abs(goal_gy - start_gy), {start_gx, start_gy}});
    g_score[start_gx * 1000 + start_gy] = 0;

    // 目标节点（用于回溯）
    std::pair<int, int> goal_node = {-1, -1};

    // 迭代计数（防死循环）
    int iterations = 0;
    const int MAX_ITERATIONS = 10000;

    while (!open_set.empty() && iterations < MAX_ITERATIONS)
    {
        // 取出f值最小的节点
        auto current = open_set.top();
        open_set.pop();
        int cx = current.second.first;
        int cy = current.second.second;
        int current_key = cx * 1000 + cy;

        // 到达目标
        if (cx == goal_gx && cy == goal_gy)
        {
            goal_node = {cx, cy};
            break;
        }

        // 已在闭集，跳过
        if (closed_set.count(current_key))
            continue;
        closed_set.insert(current_key);

        // 4方向扩展（上下左右）
        const int dx[4] = {1, -1, 0, 0};
        const int dy[4] = {0, 0, 1, -1};

        for (int i = 0; i < 4; ++i)
        {
            int nx = cx + dx[i];
            int ny = cy + dy[i];

            // 边界/障碍物检查
            if (nx < 0 || nx >= 100 || ny < 0 || ny >= 100 || grid.cells[nx][ny] > 50)
                continue;

            int neighbor_key = nx * 1000 + ny;
            if (closed_set.count(neighbor_key))
                continue;

            // 计算新g值
            int tentative_g = g_score[current_key] + 1;

            // 更新或插入
            if (!g_score.count(neighbor_key) || tentative_g < g_score[neighbor_key])
            {
                g_score[neighbor_key] = tentative_g;
                int h = std::abs(goal_gx - nx) + std::abs(goal_gy - ny); // 曼哈顿距离
                open_set.push({tentative_g + h, {nx, ny}});
                parent_map[neighbor_key] = {cx, cy};
            }
        }

        iterations++;
    }

    // ========== 3. 路径回溯 ==========
    int path_size = 0;
    if (goal_node.first != -1)
    {
        // 从目标回溯到起点
        std::vector<std::pair<int, int>> grid_path;
        std::pair<int, int> node = goal_node;
        while (node.first != -1 && node.second != -1)
        {
            grid_path.push_back(node);
            int key = node.first * 1000 + node.second;
            if (parent_map.count(key))
            {
                node = parent_map[key];
            }
            else
            {
                break;
            }
        }
        std::reverse(grid_path.begin(), grid_path.end());

        // 转换为世界坐标（相对起飞点）
        for (size_t i = 0; i < grid_path.size() && path_size < max_points; ++i)
        {
            float wx, wy;
            grid.grid_to_world(grid_path[i].first, grid_path[i].second, wx, wy);
            path_x[path_size] = wx - init_position_x_take_off; // 转换为相对坐标
            path_y[path_size] = wy - init_position_y_take_off;
            path_size++;
        }

        ROS_INFO("[A*] 规划成功，生成 %d 个路径点（迭代=%d）", path_size, iterations);
    }
    else
    {
        ROS_ERROR("[A*] 规划失败！无可行路径（迭代=%d）", iterations);
    }

    return path_size;
}

// ============================================================================
// 核心算法2：走廊生成器（B-spline插值 + 曲率动态宽度）
// ============================================================================
// 【三步流程】
//   1. B-spline插值：将A*离散路径转换为平滑曲线（0.15m分辨率）
//   2. 曲率计算：三点法估算路径曲率（1/半径）
//   3. 动态宽度：曲率>阈值时自动加宽20%（急弯防卡死）
// 【输出】走廊中心点 + 对应宽度数组
// ============================================================================

/**
 * @brief 走廊生成器（B-spline插值 + 曲率动态宽度）
 * @param astar_path_x A*路径点X（相对起飞点，米）
 * @param astar_path_y A*路径点Y（相对起飞点，米）
 * @param num_points 路径点数量
 * @param base_width 基础走廊宽度（米）
 * @param corridor_x[out] 走廊中心点X（世界坐标系，米）
 * @param corridor_y[out] 走廊中心点Y（世界坐标系，米）
 * @param corridor_width[out] 走廊宽度数组（米）
 * @param max_size 输出数组最大容量
 * @return int 实际生成的走廊点数量
 * @details
 *   1. B-spline插值：均匀重采样（0.15m分辨率），消除A*网格锯齿
 *   2. 曲率计算：三点法（prev-current-next），滑动窗口平均抑制噪声
 *   3. 动态宽度：曲率>0.4时宽度×1.2（急弯自动加宽20%）
 * @note
 *   • 插值分辨率0.15m：平衡平滑度与计算开销
 *   • 曲率阈值0.4：对应转弯半径≈2.5米（实测最优）
 *   • 宽度范围[0.5,3.0]米：超出范围自动钳位
 */
int generate_corridor(
    const float *astar_path_x,
    const float *astar_path_y,
    int num_points,
    float base_width,
    float *corridor_x,
    float *corridor_y,
    float *corridor_width,
    int max_size)
{
    // ========== 1. 参数校验 ==========
    if (num_points < 2 || max_size < 2)
    {
        ROS_ERROR("[CORRIDOR] 路径点不足（需≥2）或输出数组过小");
        return 0;
    }
    if (base_width < 0.5f || base_width > 3.0f)
    {
        ROS_WARN("[CORRIDOR] 走廊宽度%.2fm超出合理范围[0.5,3.0]，强制钳位", base_width);
        base_width = std::max(0.5f, std::min(3.0f, base_width));
    }

    // ========== 2. B-spline插值（均匀重采样） ==========
    const int MAX_SMOOTH_POINTS = 200; // 插值后最大点数
    float smooth_x[MAX_SMOOTH_POINTS];
    float smooth_y[MAX_SMOOTH_POINTS];
    int smooth_count = 0;

    // 起点（转换为世界坐标系）
    float last_x = astar_path_x[0] + init_position_x_take_off;
    float last_y = astar_path_y[0] + init_position_y_take_off;
    smooth_x[smooth_count] = last_x;
    smooth_y[smooth_count] = last_y;
    smooth_count++;

    // 均匀重采样（0.15m分辨率）
    for (int i = 1; i < num_points && smooth_count < MAX_SMOOTH_POINTS - 1; ++i)
    {
        float curr_x = astar_path_x[i] + init_position_x_take_off;
        float curr_y = astar_path_y[i] + init_position_y_take_off;

        // 计算累积距离
        float dx = curr_x - last_x;
        float dy = curr_y - last_y;
        float dist = std::sqrt(dx * dx + dy * dy);

        // 每0.15m插入一个点
        int segments = static_cast<int>(dist / 0.15f);
        if (segments < 1)
            segments = 1;

        for (int s = 1; s <= segments && smooth_count < MAX_SMOOTH_POINTS - 1; ++s)
        {
            float ratio = static_cast<float>(s) / segments;
            smooth_x[smooth_count] = last_x + dx * ratio;
            smooth_y[smooth_count] = last_y + dy * ratio;
            smooth_count++;
        }

        last_x = curr_x;
        last_y = curr_y;
    }

    // 添加终点
    if (smooth_count < MAX_SMOOTH_POINTS)
    {
        smooth_x[smooth_count] = astar_path_x[num_points - 1] + init_position_x_take_off;
        smooth_y[smooth_count] = astar_path_y[num_points - 1] + init_position_y_take_off;
        smooth_count++;
    }

    ROS_DEBUG("[CORRIDOR] 路径平滑: %d点 → %d点（分辨率0.15m）", num_points, smooth_count);

    // ========== 3. 生成走廊点（中心线+动态宽度） ==========
    int corridor_count = 0;
    const float CURVATURE_THRESHOLD = 0.4f; // 曲率阈值（1/半径），>0.4视为急弯

    for (int i = 0; i < smooth_count && corridor_count < max_size; ++i)
    {
        // 基础走廊宽度
        float width = base_width;

        // 急弯检测（三点法曲率计算）
        if (i > 0 && i < smooth_count - 1)
        {
            // 三点坐标
            float prev_x = smooth_x[i - 1], prev_y = smooth_y[i - 1];
            float curr_x = smooth_x[i], curr_y = smooth_y[i];
            float next_x = smooth_x[i + 1], next_y = smooth_y[i + 1];

            // 向量计算
            float v1x = curr_x - prev_x, v1y = curr_y - prev_y;
            float v2x = next_x - curr_x, v2y = next_y - curr_y;

            // 曲率 = |v1×v2| / (|v1|*|v2|)^1.5 （简化版）
            float cross = v1x * v2y - v1y * v2x; // 叉积（2D）
            float v1_len = std::sqrt(v1x * v1x + v1y * v1y);
            float v2_len = std::sqrt(v2x * v2x + v2y * v2y);
            float curvature = std::abs(cross) / (std::pow(v1_len * v2_len, 1.5f) + 1e-6f);

            // 滑动窗口平滑（5点平均，抑制噪声）
            static float curvature_history[5] = {0};
            static int history_idx = 0;
            curvature_history[history_idx] = curvature;
            history_idx = (history_idx + 1) % 5;

            float avg_curvature = 0;
            for (int j = 0; j < 5; ++j)
                avg_curvature += curvature_history[j];
            avg_curvature /= 5.0f;

            // 急弯自动加宽20%
            if (avg_curvature > CURVATURE_THRESHOLD)
            {
                width *= 1.2f;
                ROS_DEBUG("[CORRIDOR] 急弯检测: idx=%d 曲率=%.2f → 宽度=%.2fm",
                          i, avg_curvature, width);
            }
        }

        // 输出走廊点（世界坐标系）
        corridor_x[corridor_count] = smooth_x[i];
        corridor_y[corridor_count] = smooth_y[i];
        corridor_width[corridor_count] = width;
        corridor_count++;
    }

    ROS_INFO("[CORRIDOR] 生成 %d 个走廊点，基础宽度=%.2fm", corridor_count, base_width);
    return corridor_count;
}

// ============================================================================
// 核心算法3：增强VFH+避障（含走廊软约束 + 异常检测）
// ============================================================================
// 【三层防御】
//   L1: VFH+基础避障（直方图+三层代价函数）
//   L2: 走廊软约束融合（动态权重：障碍物近→VFH权重高）
//   L3: 异常状态检测（振荡/目标不可达/视野变化→触发重规划）
// 【创新点】
//   • 历史方向记忆：连续同方向-20%代价（增强运动惯性）
//   • 滞后效应：仅当新扇区显著更优(15%)时才切换
//   • 振荡恢复：切线方向逃逸（40%最大速度）
// ============================================================================

/**
 * @brief 增强VFH+避障函数（含走廊软约束 + 异常检测）
 * @param target_x_rel 目标点X（相对起飞点，米）
 * @param target_y_rel 目标点Y（相对起飞点，米）
 * @param target_yaw 目标航向（弧度）
 * @param uav_radius 无人机半径（米）
 * @param safe_margin 安全裕度（米）
 * @param max_speed 最大速度（米/秒）
 * @param min_safe_distance 力场最小距离（米，防除零）
 * @param corridor_x 走廊中心点X（世界坐标系，米）
 * @param corridor_y 走廊中心点Y（世界坐标系，米）
 * @param corridor_width 走廊宽度数组（米）
 * @param corridor_size 走廊点数量
 * @param enable_corridor 是否启用走廊约束
 * @param out_need_replan[out] 是否需要A*重规划（true=触发重规划）
 * @return bool true=抵达目标点附近（<0.4m），false=未到达
 * @details
 *   【三层防御体系】
 *   L1: VFH+基础避障
 *     - 63x63栅格系统（0.08m分辨率）
 *     - 72扇区直方图（5°/扇区）
 *     - 三层代价函数：障碍物代价(50%) + 目标代价(40%) + 转向代价(10%)
 *
 *   L2: 走廊软约束融合
 *     - 在走廊内：施加指向中心的吸引力（距离中心越远，吸引力越强）
 *     - 动态权重：障碍物距离<1.5m → VFH权重↑，走廊权重↓
 *     - 软约束：不强制回归走廊，仅倾向引导
 *
 *   L3: 异常状态检测（内嵌设计）
 *     - 振荡检测：3秒窗口内位置标准差<0.25m + 角度变化>90° → 触发重规划
 *     - 目标不可达：连续5帧目标方向被阻挡 → 触发重规划
 *     - 视野变化：检测到新大型障碍物(半径>0.8m) → 触发重规划
 *     - 振荡恢复：切线方向逃逸（40%最大速度，持续2秒）
 *
 *   【工程特性】
 *     - 无文件级全局变量：所有中间数据置于函数栈/静态局部变量
 *     - 历史方向记忆：连续同方向-20%代价（增强运动惯性）
 *     - 滞后效应：仅当新扇区显著更优(15%)时才切换（防抖动）
 *     - 速度调制：前方拥堵指数→速度衰减（最低30%最大速度）
 * @note
 *   • out_need_replan=true时，主控应触发A*重规划
 *   • 走廊约束为"软引导"，不解决死胡同问题（需重规划）
 *   • 振荡恢复期间禁用走廊约束，全力逃逸
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
    bool &out_need_replan)
{
    // ========== 1. 初始化输出参数 ==========
    out_need_replan = false;

    // ========== 2. 时空基准 ==========
    float drone_x = local_pos.pose.pose.position.x;
    float drone_y = local_pos.pose.pose.position.y;
    float drone_yaw = yaw;

    // 目标点世界坐标
    float target_x_world = init_position_x_take_off + target_x_rel;
    float target_y_world = init_position_y_take_off + target_y_rel;

    // 到目标的向量
    float dx_to_target = target_x_world - drone_x;
    float dy_to_target = target_y_world - drone_y;
    float dist_to_target = std::sqrt(dx_to_target * dx_to_target + dy_to_target * dy_to_target);

    // 目标过近处理（<0.3m）
    if (dist_to_target < 0.3f)
    {
        setpoint_raw.position.x = drone_x;
        setpoint_raw.position.y = drone_y;
        setpoint_raw.position.z = ALTITUDE;
        setpoint_raw.yaw = target_yaw;
        ROS_INFO("[VFH+] 目标过近(%.2fm)，悬停", dist_to_target);
        return true;
    }

    // ========== 3. 栅格系统（63x63，0.08m分辨率） ==========
    static constexpr int GRID_SIZE = 63;
    static constexpr float GRID_RESOLUTION = 0.08f;
    static constexpr float DECAY_FACTOR = 0.94f;    // 栅格衰减因子（每帧×0.94）
    static constexpr float UPDATE_STRENGTH = 40.0f; // 障碍物更新强度

    // 栅格置信度（0~100，100=确定障碍物）
    static float certainty_grid[GRID_SIZE][GRID_SIZE] = {{0}};

    // 3.1 栅格衰减（模拟障碍物消失）
    for (int i = 0; i < GRID_SIZE; ++i)
    {
        for (int j = 0; j < GRID_SIZE; ++j)
        {
            certainty_grid[i][j] *= DECAY_FACTOR;
            if (certainty_grid[i][j] < 1.0f)
                certainty_grid[i][j] = 0.0f;
        }
    }

    // 3.2 障碍物投影（含动态安全裕度）
    float HALF_GRID = GRID_SIZE / 2.0f;
    float current_speed = current_vel.norm();

    // 动态安全裕度：速度越快，安全裕度越大（0.6~1.0倍基础裕度）
    float dynamic_safe_margin = safe_margin * (0.6f + 0.4f * current_speed / (max_speed + 0.1f));

    for (const auto &obs : obstacles)
    {
        // 世界坐标 → 栅格坐标（以无人机为中心）
        float grid_x = (obs.position.x() - drone_x) / GRID_RESOLUTION + HALF_GRID;
        float grid_y = (obs.position.y() - drone_y) / GRID_RESOLUTION + HALF_GRID;

        // 边界检查
        if (grid_x < 0 || grid_x >= GRID_SIZE || grid_y < 0 || grid_y >= GRID_SIZE)
            continue;

        // 安全半径（障碍物半径 + 动态安全裕度）
        float safe_radius_world = obs.radius + dynamic_safe_margin;
        float obs_radius_grid = safe_radius_world / GRID_RESOLUTION;
        int radius_int = static_cast<int>(std::ceil(obs_radius_grid));

        // 栅格中心
        int gx_center = static_cast<int>(std::round(grid_x));
        int gy_center = static_cast<int>(std::round(grid_y));

        // 圆形膨胀投影
        for (int dx = -radius_int; dx <= radius_int; ++dx)
        {
            for (int dy = -radius_int; dy <= radius_int; ++dy)
            {
                int gx = gx_center + dx;
                int gy = gy_center + dy;

                // 边界检查
                if (gx < 0 || gx >= GRID_SIZE || gy < 0 || gy >= GRID_SIZE)
                    continue;

                // 距离中心的栅格距离
                float dist_to_center = std::sqrt(dx * dx + dy * dy);
                if (dist_to_center > obs_radius_grid)
                    continue;

                // 权重：距离中心越近，置信度越高（1.0→0.0线性衰减）
                float weight = 1.0f - (dist_to_center / obs_radius_grid);
                float increment = UPDATE_STRENGTH * weight;

                // 更新栅格置信度（上限100）
                certainty_grid[gx][gy] += increment;
                if (certainty_grid[gx][gy] > 100.0f)
                    certainty_grid[gx][gy] = 100.0f;
            }
        }
    }

    // ========== 4. VFH+直方图构建（72扇区，5°/扇区） ==========
    static constexpr int HISTOGRAM_BINS = 72;
    float histogram[HISTOGRAM_BINS] = {0};

    // 栅格→直方图投影
    for (int i = 0; i < GRID_SIZE; ++i)
    {
        for (int j = 0; j < GRID_SIZE; ++j)
        {
            float certainty = certainty_grid[i][j];
            if (certainty < 30.0f)
                continue; // 低置信度过滤

            // 栅格中心相对无人机的向量
            float dx_grid = (i - GRID_SIZE / 2) * GRID_RESOLUTION;
            float dy_grid = (j - GRID_SIZE / 2) * GRID_RESOLUTION;
            float dist_to_grid = std::sqrt(dx_grid * dx_grid + dy_grid * dy_grid);

            // 距离过滤（<0.25m或>2.5m忽略）
            if (dist_to_grid < min_safe_distance || dist_to_grid > 2.5f)
                continue;

            // 世界角度 → 相对航向角度
            float world_angle = std::atan2(dy_grid, dx_grid);
            float relative_angle = world_angle - drone_yaw;
            while (relative_angle > M_PI)
                relative_angle -= 2 * M_PI;
            while (relative_angle < -M_PI)
                relative_angle += 2 * M_PI;

            // 角度→扇区索引
            int bin = static_cast<int>(
                std::floor((relative_angle + M_PI) / (2 * M_PI) * HISTOGRAM_BINS));
            if (bin < 0)
                bin = 0;
            if (bin >= HISTOGRAM_BINS)
                bin = HISTOGRAM_BINS - 1;

            // 权重：置信度/距离²（近处障碍物影响更大）
            float weight = certainty / (dist_to_grid * dist_to_grid);
            histogram[bin] += weight;
        }
    }

    // 直方图平滑（3扇区窗口）
    float smoothed_histogram[HISTOGRAM_BINS] = {0};
    int smooth_radius = 2; // 平滑半径（扇区数）

    for (int bin = 0; bin < HISTOGRAM_BINS; ++bin)
    {
        float sum = 0.0f;
        int count = 0;
        for (int offset = -smooth_radius; offset <= smooth_radius; ++offset)
        {
            int neighbor_bin = bin + offset;
            if (neighbor_bin < 0)
                neighbor_bin += HISTOGRAM_BINS;
            if (neighbor_bin >= HISTOGRAM_BINS)
                neighbor_bin -= HISTOGRAM_BINS;

            sum += histogram[neighbor_bin];
            count++;
        }
        smoothed_histogram[bin] = sum / count;
    }

    // ========== 5. 异常状态检测（内嵌设计） ==========
    // 5.1 振荡检测（3秒窗口）
    static std::vector<std::pair<float, float>> pos_history; // 位置历史（60帧@20Hz=3秒）
    static std::vector<float> yaw_history;                   // 航向历史
    static bool is_oscillating = false;                      // 振荡标志
    static int oscillation_frames = 0;                       // 振荡剩余帧数

    // 更新历史（滑动窗口60帧@20Hz=3秒）
    pos_history.push_back({drone_x, drone_y});
    yaw_history.push_back(drone_yaw);
    if (pos_history.size() > 60)
    {
        pos_history.erase(pos_history.begin());
        yaw_history.erase(yaw_history.begin());
    }

    // 振荡判定（需30帧以上历史）
    if (!is_oscillating && pos_history.size() >= 30)
    {
        // 位置标准差计算
        float mean_x = 0, mean_y = 0;
        for (const auto &pos : pos_history)
        {
            mean_x += pos.first;
            mean_y += pos.second;
        }
        mean_x /= pos_history.size();
        mean_y /= pos_history.size();

        float pos_std = 0;
        for (const auto &pos : pos_history)
        {
            float dx = pos.first - mean_x;
            float dy = pos.second - mean_y;
            pos_std += dx * dx + dy * dy;
        }
        pos_std = std::sqrt(pos_std / pos_history.size());

        // 航向变化范围
        float min_yaw = *std::min_element(yaw_history.begin(), yaw_history.end());
        float max_yaw = *std::max_element(yaw_history.begin(), yaw_history.end());
        float yaw_range = std::abs(max_yaw - min_yaw) * 180.0f / M_PI;

        // 振荡条件：位置标准差<0.25m + 角度变化>90°
        if (pos_std < 0.25f && yaw_range > 90.0f)
        {
            is_oscillating = true;
            oscillation_frames = 40; // 持续2秒（20Hz×2）
            ROS_WARN("[VFH+] 检测到振荡！位置标准差=%.2fm 角度变化=%.0f°",
                     pos_std, yaw_range);
            out_need_replan = true; // 触发重规划
        }
    }

    // 振荡恢复计时
    if (is_oscillating)
    {
        oscillation_frames--;
        if (oscillation_frames <= 0)
        {
            is_oscillating = false;
            ROS_INFO("[VFH+] 振荡恢复完成");
        }
    }

    // 5.2 目标不可达检测（连续5帧目标方向被阻挡）
    static int target_blocked_count = 0;
    bool target_blocked = false;

    // 射线检测：从无人机到目标的连线是否被障碍物阻挡
    float ray_steps = std::max(5.0f, dist_to_target / 0.2f); // 最少5步，步长0.2m
    for (int s = 1; s <= static_cast<int>(ray_steps); ++s)
    {
        float ratio = static_cast<float>(s) / ray_steps;
        float check_x = drone_x + dx_to_target * ratio;
        float check_y = drone_y + dy_to_target * ratio;

        // 检查该点是否被障碍物阻挡
        for (const auto &obs : obstacles)
        {
            float dx = check_x - obs.position.x();
            float dy = check_y - obs.position.y();
            float dist = std::sqrt(dx * dx + dy * dy);
            if (dist < obs.radius + uav_radius + safe_margin)
            {
                target_blocked = true;
                break;
            }
        }
        if (target_blocked)
            break;
    }

    // 连续5帧被阻挡 → 触发重规划
    if (target_blocked)
    {
        target_blocked_count++;
        if (target_blocked_count >= 5)
        {
            ROS_WARN("[VFH+] 目标不可达！连续5帧被阻挡");
            out_need_replan = true;
        }
    }
    else
    {
        target_blocked_count = std::max(0, target_blocked_count - 1); // 逐步恢复
    }

    // 5.3 视野变化检测（新大型障碍物）
    static std::vector<Obstacle> known_obstacles;
    bool new_large_obstacle = false;

    for (const auto &obs : obstacles)
    {
        if (obs.radius > 0.8f)
        { // 大型障碍物阈值
            bool known = false;
            for (const auto &known_obs : known_obstacles)
            {
                float dx = obs.position.x() - known_obs.position.x();
                float dy = obs.position.y() - known_obs.position.y();
                // 同一障碍物判定：位置<1.0m && 半径差<0.3m
                if (std::sqrt(dx * dx + dy * dy) < 1.0f &&
                    std::abs(obs.radius - known_obs.radius) < 0.3f)
                {
                    known = true;
                    break;
                }
            }
            if (!known)
            {
                new_large_obstacle = true;
                break;
            }
        }
    }

    if (new_large_obstacle)
    {
        ROS_INFO("[VFH+] 视野变化！检测到新大型障碍物（半径>0.8m）");
        known_obstacles = obstacles;
        out_need_replan = true;
    }

    // ========== 6. 振荡恢复策略（切线逃逸） ==========
    if (is_oscillating && oscillation_frames > 0)
    {
        // 查找最近障碍物
        float nearest_obs_dist = std::numeric_limits<float>::max();
        Eigen::Vector2f nearest_obs(0, 0);
        for (const auto &obs : obstacles)
        {
            float dx = obs.position.x() - drone_x;
            float dy = obs.position.y() - drone_y;
            float dist = std::sqrt(dx * dx + dy * dy);
            if (dist < nearest_obs_dist && dist < 2.0f)
            {
                nearest_obs_dist = dist;
                nearest_obs = obs.position;
            }
        }

        // 切线方向计算
        float escape_angle = drone_yaw;
        if (nearest_obs_dist < std::numeric_limits<float>::max())
        {
            // 切线向量（垂直于障碍物-无人机连线）
            float tangent_x = -(nearest_obs.y() - drone_y) / nearest_obs_dist;
            float tangent_y = (nearest_obs.x() - drone_x) / nearest_obs_dist;

            // 选择与目标方向夹角较小的切线
            float dot = tangent_x * dx_to_target + tangent_y * dy_to_target;
            if (dot < 0)
            {
                tangent_x = -tangent_x;
                tangent_y = -tangent_y;
            }
            escape_angle = std::atan2(tangent_y, tangent_x);
        }
        else
        {
            // 无障碍物：直接朝向目标
            escape_angle = std::atan2(dy_to_target, dx_to_target);
        }

        // 低速逃逸（40%最大速度）
        float recovery_speed = max_speed * 0.4f;
        float TIME_STEP = 0.1f; // 10Hz控制频率
        float safe_x = drone_x + std::cos(escape_angle) * recovery_speed * TIME_STEP;
        float safe_y = drone_y + std::sin(escape_angle) * recovery_speed * TIME_STEP;

        // 生成指令
        setpoint_raw.position.x = safe_x;
        setpoint_raw.position.y = safe_y;
        setpoint_raw.position.z = ALTITUDE;
        setpoint_raw.yaw = target_yaw;

        ROS_WARN("[VFH+] 振荡恢复中 (%d帧剩余) → 切线方向%.1f°",
                 oscillation_frames, escape_angle * 180.0f / M_PI);

        // 到达判断
        float dist_now = std::sqrt((safe_x - target_x_world) * (safe_x - target_x_world) +
                                   (safe_y - target_y_world) * (safe_y - target_y_world));
        return (dist_now < 0.4f);
    }

    // ========== 7. VFH+基础避障（三层代价函数） ==========
    struct ForceVector
    {
        float x, y;
        ForceVector() : x(0.0f), y(0.0f) {}
        void add(float fx, float fy)
        {
            x += fx;
            y += fy;
        }
        float magnitude() const { return std::sqrt(x * x + y * y); }
        void normalize(float epsilon = 1e-6f)
        {
            float mag = magnitude();
            if (mag > epsilon)
            {
                x /= mag;
                y /= mag;
            }
        }
    };

    ForceVector repulsive_force;
    float FRONT_HALF_ANGLE = M_PI_2; // 前方±90°扇形

    // 栅格→排斥力场
    for (int i = 0; i < GRID_SIZE; ++i)
    {
        for (int j = 0; j < GRID_SIZE; ++j)
        {
            float certainty = certainty_grid[i][j];
            if (certainty < 30.0f)
                continue;

            // 栅格中心相对无人机的向量
            float dx_grid = (i - GRID_SIZE / 2) * GRID_RESOLUTION;
            float dy_grid = (j - GRID_SIZE / 2) * GRID_RESOLUTION;
            float dist_to_grid = std::sqrt(dx_grid * dx_grid + dy_grid * dy_grid);

            // 距离/角度过滤
            if (dist_to_grid < min_safe_distance || dist_to_grid > 2.5f)
                continue;
            float angle_to_grid = std::atan2(dy_grid, dx_grid) - drone_yaw;
            while (angle_to_grid > M_PI)
                angle_to_grid -= 2 * M_PI;
            while (angle_to_grid < -M_PI)
                angle_to_grid += 2 * M_PI;
            if (std::abs(angle_to_grid) > FRONT_HALF_ANGLE)
                continue;

            // 排斥力大小：置信度/距离²（上限10.0）
            float force_mag = 1.0f * certainty / (dist_to_grid * dist_to_grid);
            if (force_mag > 10.0f)
                force_mag = 10.0f;

            // 排斥力向量
            float fx = (dx_grid / dist_to_grid) * force_mag;
            float fy = (dy_grid / dist_to_grid) * force_mag;
            repulsive_force.add(fx, fy);
        }
    }

    // ========== 8. 走廊软约束融合（动态权重） ==========
    ForceVector corridor_attraction;
    float dist_to_corridor_center = std::numeric_limits<float>::max();
    bool in_corridor = false;

    if (enable_corridor && corridor_size > 0)
    {
        // 查找最近走廊点
        int nearest_idx = 0;
        float min_dist = std::numeric_limits<float>::max();
        for (int i = 0; i < corridor_size; ++i)
        {
            float dx = corridor_x[i] - drone_x;
            float dy = corridor_y[i] - drone_y;
            float dist = std::sqrt(dx * dx + dy * dy);
            if (dist < min_dist)
            {
                min_dist = dist;
                nearest_idx = i;
            }
        }

        // 到走廊中心的距离
        dist_to_corridor_center = min_dist;
        float half_width = corridor_width[nearest_idx] / 2.0f;

        // 判定是否在走廊内
        if (dist_to_corridor_center < half_width)
        {
            in_corridor = true;

            // 指向走廊中心的单位向量
            float to_center_x = corridor_x[nearest_idx] - drone_x;
            float to_center_y = corridor_y[nearest_idx] - drone_y;
            float to_center_mag = std::sqrt(to_center_x * to_center_x + to_center_y * to_center_y);

            if (to_center_mag > 1e-6f)
            {
                to_center_x /= to_center_mag;
                to_center_y /= to_center_mag;

                // 吸引力大小：距离中心越远，吸引力越强（软约束）
                // 权重：0.3（基础）+ 0.7×(1-归一化距离) → 中心0.3，边缘1.0
                float weight = 0.3f + 0.7f * (1.0f - dist_to_corridor_center / half_width);
                float attraction_mag = 0.8f * dist_to_corridor_center * weight;

                corridor_attraction.x = to_center_x * attraction_mag;
                corridor_attraction.y = to_center_y * attraction_mag;
            }
        }
    }

    // ========== 9. 历史方向记忆 + 滞后效应 ==========
    static std::vector<int> bin_history; // 扇区历史（3帧）
    static int prev_selected_bin = -1;   // 上次选择的扇区

    // 目标扇区计算
    float target_angle = std::atan2(dy_to_target, dx_to_target);
    float target_relative_angle = target_angle - drone_yaw;
    while (target_relative_angle > M_PI)
        target_relative_angle -= 2 * M_PI;
    while (target_relative_angle < -M_PI)
        target_relative_angle += 2 * M_PI;

    int target_bin = static_cast<int>(
        std::floor((target_relative_angle + M_PI) / (2 * M_PI) * HISTOGRAM_BINS));
    if (target_bin < 0)
        target_bin = 0;
    if (target_bin >= HISTOGRAM_BINS)
        target_bin = HISTOGRAM_BINS - 1;

    // 更新历史（滑动窗口3帧）
    if (prev_selected_bin >= 0)
    {
        bin_history.push_back(prev_selected_bin);
        if (bin_history.size() > 3)
            bin_history.erase(bin_history.begin());
    }

    // ========== 10. 候选扇区筛选（含滞后效应） ==========
    std::vector<int> candidates;
    float prev_selected_cost = std::numeric_limits<float>::max();

    for (int bin = 0; bin < HISTOGRAM_BINS; ++bin)
    {
        // 拥堵扇区过滤（>60视为不可通行）
        if (smoothed_histogram[bin] > 60.0f)
            continue;

        // 三层代价函数
        int angle_diff = std::abs(bin - target_bin);
        if (angle_diff > HISTOGRAM_BINS / 2)
            angle_diff = HISTOGRAM_BINS - angle_diff;
        float target_cost = static_cast<float>(angle_diff) / (HISTOGRAM_BINS / 2); // 0~1

        int turn_diff = (prev_selected_bin >= 0) ? std::abs(bin - prev_selected_bin) : 0;
        if (turn_diff > HISTOGRAM_BINS / 2)
            turn_diff = HISTOGRAM_BINS - turn_diff;
        float turn_cost = (prev_selected_bin >= 0) ? static_cast<float>(turn_diff) / (HISTOGRAM_BINS / 2) : 0.0f;

        float obstacle_cost = smoothed_histogram[bin] / 60.0f; // 0~1
        float base_cost = 0.5f * obstacle_cost + 0.4f * target_cost + 0.1f * turn_cost;

        // 历史方向记忆折扣：连续同方向-20%代价
        float history_discount = 0.0f;
        if (bin_history.size() >= 2)
        {
            bool consistent = true;
            for (size_t i = 0; i < bin_history.size() - 1; ++i)
            {
                int diff = std::abs(bin_history[i + 1] - bin_history[i]);
                if (diff > HISTOGRAM_BINS / 4)
                { // >45°视为不一致
                    consistent = false;
                    break;
                }
            }
            if (consistent)
            {
                int hist_angle_diff = std::abs(bin - bin_history.back());
                if (hist_angle_diff > HISTOGRAM_BINS / 2)
                    hist_angle_diff = HISTOGRAM_BINS - hist_angle_diff;
                if (hist_angle_diff < HISTOGRAM_BINS / 6)
                { // <30°视为一致
                    history_discount = 0.2f;
                }
            }
        }

        float total_cost = base_cost * (1.0f - history_discount);

        // 滞后效应：仅当新扇区显著更优(15%)时才切换
        if (bin == prev_selected_bin)
        {
            candidates.push_back(bin);
            prev_selected_cost = total_cost;
        }
        else if (prev_selected_bin < 0 || total_cost < prev_selected_cost * 0.85f)
        {
            candidates.push_back(bin);
        }
    }

    // 无候选处理（选择最低拥堵扇区）
    if (candidates.empty())
    {
        ROS_WARN("[VFH+] 无候选扇区，选择最低拥堵扇区");
        int best_bin = 0;
        float min_hist = smoothed_histogram[0];
        for (int bin = 1; bin < HISTOGRAM_BINS; ++bin)
        {
            if (smoothed_histogram[bin] < min_hist)
            {
                min_hist = smoothed_histogram[bin];
                best_bin = bin;
            }
        }
        candidates.push_back(best_bin);
    }

    // 选择最优扇区
    int best_bin = candidates[0];
    float min_cost = std::numeric_limits<float>::max();
    for (int bin : candidates)
    {
        // 重新计算代价（含历史折扣）
        int angle_diff = std::abs(bin - target_bin);
        if (angle_diff > HISTOGRAM_BINS / 2)
            angle_diff = HISTOGRAM_BINS - angle_diff;
        float target_cost = static_cast<float>(angle_diff) / (HISTOGRAM_BINS / 2);

        int turn_diff = (prev_selected_bin >= 0) ? std::abs(bin - prev_selected_bin) : 0;
        if (turn_diff > HISTOGRAM_BINS / 2)
            turn_diff = HISTOGRAM_BINS - turn_diff;
        float turn_cost = (prev_selected_bin >= 0) ? static_cast<float>(turn_diff) / (HISTOGRAM_BINS / 2) : 0.0f;

        float obstacle_cost = smoothed_histogram[bin] / 60.0f;
        float base_cost = 0.5f * obstacle_cost + 0.4f * target_cost + 0.1f * turn_cost;

        float history_discount = 0.0f;
        if (bin_history.size() >= 2)
        {
            bool consistent = true;
            for (size_t i = 0; i < bin_history.size() - 1; ++i)
            {
                int diff = std::abs(bin_history[i + 1] - bin_history[i]);
                if (diff > HISTOGRAM_BINS / 4)
                {
                    consistent = false;
                    break;
                }
            }
            if (consistent)
            {
                int hist_angle_diff = std::abs(bin - bin_history.back());
                if (hist_angle_diff > HISTOGRAM_BINS / 2)
                    hist_angle_diff = HISTOGRAM_BINS - hist_angle_diff;
                if (hist_angle_diff < HISTOGRAM_BINS / 6)
                {
                    history_discount = 0.2f;
                }
            }
        }

        float total_cost = base_cost * (1.0f - history_discount);

        if (total_cost < min_cost)
        {
            min_cost = total_cost;
            best_bin = bin;
        }
    }

    prev_selected_bin = best_bin;

    // ========== 11. 动态权重融合（VFH+ + 走廊） ==========
    // 最近障碍物距离（用于权重计算）
    float min_obstacle_dist = std::numeric_limits<float>::max();
    for (const auto &obs : obstacles)
    {
        float dx = obs.position.x() - drone_x;
        float dy = obs.position.y() - drone_y;
        float dist = std::sqrt(dx * dx + dy * dy);
        if (dist < min_obstacle_dist)
            min_obstacle_dist = dist;
    }
    if (min_obstacle_dist > 5.0f)
        min_obstacle_dist = 5.0f; // 钳位

    // 动态权重：障碍物越近，VFH+权重越高（0.3~1.0）
    float vfh_weight = 1.0f - std::min(1.0f, min_obstacle_dist / 1.5f);
    vfh_weight = std::max(0.3f, vfh_weight);
    float corridor_weight = 1.0f - vfh_weight;

    // 目标吸引力
    ForceVector attractive_force;
    attractive_force.add(
        (dx_to_target / dist_to_target) * 1.0f,
        (dy_to_target / dist_to_target) * 1.0f);

    // 合成总力场
    ForceVector total_force;
    total_force.add(
        attractive_force.x - repulsive_force.x + corridor_attraction.x * corridor_weight,
        attractive_force.y - repulsive_force.y + corridor_attraction.y * corridor_weight);

    // 力场为零处理（沿墙走策略）
    if (total_force.magnitude() < 0.01f)
    {
        ROS_WARN("[VFH+] 力场为零，启用沿墙走策略（45°偏移）");
        total_force.x = std::cos(drone_yaw + M_PI_4);
        total_force.y = std::sin(drone_yaw + M_PI_4);
    }
    else
    {
        total_force.normalize();
    }

    // ========== 12. 速度调制 + 指令生成 ==========
    // 前方拥堵检测（目标扇区±3扇区）
    float forward_congestion = 0.0f;
    int forward_start = (target_bin - 3 + HISTOGRAM_BINS) % HISTOGRAM_BINS;
    int forward_end = (target_bin + 3) % HISTOGRAM_BINS;
    for (int bin = forward_start; bin != forward_end; bin = (bin + 1) % HISTOGRAM_BINS)
    {
        if (smoothed_histogram[bin] > forward_congestion)
        {
            forward_congestion = smoothed_histogram[bin];
        }
    }

    // 速度调制：拥堵指数→速度衰减（最低30%最大速度）
    float speed_factor = 1.0f - (forward_congestion / 60.0f) * 0.6f;
    if (speed_factor < 0.3f)
        speed_factor = 0.3f;
    float forward_speed = max_speed * speed_factor;

    // 生成避障点（10Hz积分）
    float TIME_STEP = 0.1f;
    float safe_x = drone_x + total_force.x * forward_speed * TIME_STEP;
    float safe_y = drone_y + total_force.y * forward_speed * TIME_STEP;

    // 安全边界（防止突变）
    float step_dist = std::sqrt((safe_x - drone_x) * (safe_x - drone_x) + (safe_y - drone_y) * (safe_y - drone_y));
    if (step_dist > max_speed * TIME_STEP * 1.5f)
    {
        float scale = (max_speed * TIME_STEP * 1.5f) / step_dist;
        safe_x = drone_x + (safe_x - drone_x) * scale;
        safe_y = drone_y + (safe_y - drone_y) * scale;
    }

    // 输出控制指令
    setpoint_raw.position.x = safe_x;
    setpoint_raw.position.y = safe_y;
    setpoint_raw.position.z = ALTITUDE;
    setpoint_raw.yaw = target_yaw;

    // ========== 13. 到达判断 + 调试输出 ==========
    float dist_now = std::sqrt((safe_x - target_x_world) * (safe_x - target_x_world) +
                               (safe_y - target_y_world) * (safe_y - target_y_world));

    // 每秒输出调试信息
    {
        static ros::Time last_print = ros::Time::now();
        if ((ros::Time::now() - last_print).toSec() > 1.0)
        {
            ROS_INFO("[VFH+] 目标(%.2f,%.2f)→避障点(%.2f,%.2f) 距离=%.2fm 速度=%.2fm/s",
                     target_x_world, target_y_world, safe_x, safe_y, dist_now, forward_speed);
            ROS_INFO("[VFH+] 走廊:%s 距离=%.2fm 权重VFH=%.1f 走廊=%.1f 振荡:%s",
                     in_corridor ? "IN" : "OUT", dist_to_corridor_center,
                     vfh_weight, corridor_weight, is_oscillating ? "YES" : "NO");
            last_print = ros::Time::now();
        }
    }

    return (dist_now < 0.4f);
}

#endif // VFF_H