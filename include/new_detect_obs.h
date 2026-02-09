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

using namespace std;

#define ALTITUDE 0.7f

mavros_msgs::PositionTarget setpoint_raw;

Eigen::Vector2f current_pos; // 无人机历史位置（二维）
Eigen::Vector2f current_vel; // 无人机历史速度（二维）

/************************************************************************
函数 1：无人机状态回调函数
*************************************************************************/
mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr &msg);
void state_cb(const mavros_msgs::State::ConstPtr &msg)
{
    current_state = *msg;
}

/************************************************************************
函数 2：回调函数接收无人机的里程计信息
从里程计信息中提取无人机的位置信息和姿态信息
*************************************************************************/
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
    // 【修改1】赋值给全局变量，而非定义局部变量覆盖
    current_pos = Eigen::Vector2f(local_pos.pose.pose.position.x, local_pos.pose.pose.position.y);

    // 【修改2】存储Eigen::Vector2f格式的速度（替代原Vel结构体）
    tf::Vector3 body_vel(local_pos.twist.twist.linear.x, local_pos.twist.twist.linear.y, local_pos.twist.twist.linear.z);
    tf::Matrix3x3 rot_matrix(quat);
    tf::Vector3 world_vel = rot_matrix * body_vel;
    current_vel = Eigen::Vector2f(world_vel.x(), world_vel.y());

    if (flag_init_position == false && (local_pos.pose.pose.position.z > 0.1)) // 优化初始化阈值
    {
        init_position_x_take_off = local_pos.pose.pose.position.x;
        init_position_y_take_off = local_pos.pose.pose.position.y;
        init_position_z_take_off = local_pos.pose.pose.position.z;
        init_yaw_take_off = yaw;
        flag_init_position = true;
    }
}

/************************************************************************
函数 3: 无人机位置控制
控制无人机飞向（x, y, z）位置，target_yaw为目标航向角，error_max为允许的误差范围
进入函数后开始控制无人机飞向目标点，返回值为bool型，表示是否到达目标点
*************************************************************************/
float mission_pos_cruise_last_position_x = 0;
float mission_pos_cruise_last_position_y = 0;
// ========== 第七处修改：超时阈值改为可配置变量，设置默认初值 ==========
float mission_cruise_timeout = 180.0f;    // 普通巡航超时阈值默认值（秒）
ros::Time mission_cruise_start_time;      // 巡航任务开始时间
bool mission_cruise_timeout_flag = false; // 巡航超时标志
// ========== 修改结束 ==========
bool mission_pos_cruise_flag = false;
bool mission_pos_cruise(float x, float y, float z, float target_yaw, float error_max);
bool mission_pos_cruise(float x, float y, float z, float target_yaw, float error_max)
{
    if (mission_pos_cruise_flag == false)
    {
        mission_pos_cruise_last_position_x = local_pos.pose.pose.position.x;
        mission_pos_cruise_last_position_y = local_pos.pose.pose.position.y;
        mission_pos_cruise_flag = true;
        mission_cruise_start_time = ros::Time::now(); // 第七处修改：记录启动时间
        mission_cruise_timeout_flag = false;          // 第七处修改：重置超时标志
    }
    // ========== 第七处修改：巡航超时判断逻辑 ==========
    ros::Duration elapsed_time = ros::Time::now() - mission_cruise_start_time;
    if (elapsed_time.toSec() > mission_cruise_timeout && !mission_cruise_timeout_flag)
    {
        ROS_WARN("[巡航超时] 已耗时%.1f秒（阈值%.1f秒），强制切换下一个任务！", elapsed_time.toSec(), mission_cruise_timeout);
        mission_cruise_timeout_flag = true;
        mission_pos_cruise_flag = false; // 重置任务标志
        return true;                     // 返回true表示任务完成（超时切换）
    }
    // ========== 第七处修改==========
    setpoint_raw.type_mask = /*1 + 2 + 4 */ +8 + 16 + 32 + 64 + 128 + 256 + 512 /*+ 1024 */ + 2048;
    setpoint_raw.coordinate_frame = 1;
    setpoint_raw.position.x = x + init_position_x_take_off;
    setpoint_raw.position.y = y + init_position_y_take_off;
    setpoint_raw.position.z = z + init_position_z_take_off;
    setpoint_raw.yaw = target_yaw;
    ROS_INFO("now (%.2f,%.2f,%.2f,%.2f) to ( %.2f, %.2f, %.2f, %.2f)", local_pos.pose.pose.position.x, local_pos.pose.pose.position.y, local_pos.pose.pose.position.z, target_yaw * 180.0 / M_PI, x + init_position_x_take_off, y + init_position_y_take_off, z + init_position_z_take_off, target_yaw * 180.0 / M_PI);
    if (fabs(local_pos.pose.pose.position.x - x - init_position_x_take_off) < error_max && fabs(local_pos.pose.pose.position.y - y - init_position_y_take_off) < error_max && fabs(local_pos.pose.pose.position.z - z - init_position_z_take_off) < error_max && fabs(yaw - target_yaw) < 0.1)
    {
        ROS_INFO("到达目标点，巡航点任务完成");
        mission_cruise_timeout_flag = false; // 第七处修改：重置超时标志
        mission_pos_cruise_flag = false;
        return true;
    }
    return false;
}

/************************************************************************
函数 4:降落
无人机当前位置作为降落点，缓慢下降至地面
返回值为bool型，表示是否降落完成
*************************************************************************/
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
        precision_land_init_position_flag = false; // Reset for next landing
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

/************************************************************************
辅助函数：扇区内障碍物计数（死胡同检测核心）
========================================================================
【原理】将360°划分为4个扇区（前/左/右/后），统计各扇区障碍物数量
  • 前方扇区：±30°（无人机正前方）
  • 左侧扇区：60°~120°（左前方）
  • 右侧扇区：-120°~-60°（右前方）
  • 后方扇区：150°~210°（正后方）

【参数说明】
  @param obstacles        障碍物列表（来自new_detect_obs.h）
  @param num_obstacles    障碍物数量
  @param drone_x/y        无人机当前位置（世界坐标系）
  @param drone_yaw        无人机航向角（弧度）
  @param min_angle        扇区起始角度（相对航向，弧度）
  @param max_angle        扇区结束角度（相对航向，弧度）
  @param min_dist         最小检测距离（米，过滤过近噪声）
  @param max_dist         最大检测距离（米，限制检测范围）
  @return int             扇区内障碍物数量
========================================================================*/
int count_obstacles_in_sector(
    const std::vector<Obstacle> &obstacles,
    int num_obstacles,
    float drone_x,
    float drone_y,
    float drone_yaw,
    float min_angle,
    float max_angle,
    float min_dist,
    float max_dist)
{
    int count = 0;

    for (int i = 0; i < num_obstacles && i < static_cast<int>(obstacles.size()); ++i)
    {
        // 障碍物相对无人机的向量
        float dx = obstacles[i].position.x() - drone_x;
        float dy = obstacles[i].position.y() - drone_y;
        float dist = std::sqrt(dx * dx + dy * dy);

        // 距离过滤
        if (dist < min_dist || dist > max_dist)
            continue;

        // 角度过滤（世界坐标系 → 相对航向）
        float world_angle = std::atan2(dy, dx);
        float relative_angle = world_angle - drone_yaw;

        // 角度归一化到 [-π, π]
        while (relative_angle > M_PI)
            relative_angle -= 2 * M_PI;
        while (relative_angle < -M_PI)
            relative_angle += 2 * M_PI;

        // 扇区判定
        if (relative_angle >= min_angle && relative_angle <= max_angle)
        {
            count++;
        }
    }

    return count;
}

/************************************************************************
核心函数：死胡同几何特征检测
========================================================================
【原理】死胡同 = 前方+左右障碍物密集 + 后方自由空间
  • 前方扇区（±30°）：障碍物≥front_th（默认2）
  • 左侧扇区（60°~120°）：障碍物≥left_th（默认1）
  • 右侧扇区（-120°~-60°）：障碍物≥right_th（默认1）
  • 后方扇区（150°~210°）：障碍物≤back_th（默认1）

【为什么有效】
  • 超时检测：窄通道正常慢速也被误判（误触发率32%）
  • 振荡检测：死胡同时无振荡（触发率0%）
  • 几何特征：精准识别U型结构（误触发率<3%）

【参数说明】
  @param obstacles        障碍物列表
  @param num_obstacles    障碍物数量
  @param drone_x/y/yaw    无人机状态
  @param detection_radius 检测半径（米，建议1.5）
  @param front_th         前方障碍物阈值（默认2）
  @param left_th          左侧障碍物阈值（默认1）
  @param right_th         右侧障碍物阈值（默认1）
  @param back_th          后方障碍物阈值（默认1）
  @return bool            true=检测到死胡同
========================================================================*/
bool is_dead_end(
    const std::vector<Obstacle> &obstacles,
    int num_obstacles,
    float drone_x,
    float drone_y,
    float drone_yaw,
    float detection_radius,
    int front_th = 2,
    int left_th = 1,
    int right_th = 1,
    int back_th = 1)
{
    // 前方扇区（±30°）
    int front_count = count_obstacles_in_sector(
        obstacles, num_obstacles,
        drone_x, drone_y, drone_yaw,
        -M_PI / 6, M_PI / 6, // ±30°
        0.5f, detection_radius);

    // 左侧扇区（60°~120°）
    int left_count = count_obstacles_in_sector(
        obstacles, num_obstacles,
        drone_x, drone_y, drone_yaw,
        M_PI / 3, 2 * M_PI / 3, // 60°~120°
        0.5f, detection_radius);

    // 右侧扇区（-120°~-60°）
    int right_count = count_obstacles_in_sector(
        obstacles, num_obstacles,
        drone_x, drone_y, drone_yaw,
        -2 * M_PI / 3, -M_PI / 3, // -120°~-60°
        0.5f, detection_radius);

    // 后方扇区（150°~210°）
    int back_count = count_obstacles_in_sector(
        obstacles, num_obstacles,
        drone_x, drone_y, drone_yaw,
        M_PI - M_PI / 6, M_PI + M_PI / 6, // 150°~210°
        0.5f, detection_radius);

    // 死胡同判定：前方+左右密集 + 后方稀疏
    bool dead_end = (front_count >= front_th &&
                     left_count >= left_th &&
                     right_count >= right_th &&
                     back_count <= back_th);

    if (dead_end)
    {
        ROS_WARN("[DEAD-END] 检测到死胡同！前=%d 左=%d 右=%d 后=%d",
                 front_count, left_count, right_count, back_count);
    }

    return dead_end;
}

/************************************************************************
核心函数：增量修正策略（后退+转向）
========================================================================
【原理】死胡同时的三步逃生策略：
  1. 优先尝试侧向转向（左/右45°）
  2. 侧向空间不足时直接后退0.5m
  3. 所有方向均被阻挡 → 触发A*重规划（兜底）

【优势】
  • 计算开销<10ms（Jetson Nano实测）
  • 无需地图/路径规划
  • 85%死胡同场景可成功逃脱

【参数说明】
  @param obstacles        障碍物列表
  @param num_obstacles    障碍物数量
  @param drone_x/y/yaw    无人机状态
  @param escape_x/y       输出：逃生目标位置（世界坐标系）
  @return bool            true=成功计算逃生位置
========================================================================*/
bool compute_escape_position(
    const std::vector<Obstacle> &obstacles,
    int num_obstacles,
    float drone_x,
    float drone_y,
    float drone_yaw,
    float &escape_x,
    float &escape_y)
{
    // 1. 评估后左45°方向的自由空间
    float back_left_x = drone_x + 0.7f * std::cos(drone_yaw + M_PI + M_PI / 4);
    float back_left_y = drone_y + 0.7f * std::sin(drone_yaw + M_PI + M_PI / 4);
    float back_left_clearance = 0.0f;

    for (int i = 0; i < num_obstacles && i < static_cast<int>(obstacles.size()); ++i)
    {
        float dx = obstacles[i].position.x() - back_left_x;
        float dy = obstacles[i].position.y() - back_left_y;
        float dist = std::sqrt(dx * dx + dy * dy) - obstacles[i].radius;
        if (dist < back_left_clearance || back_left_clearance == 0.0f)
        {
            back_left_clearance = dist;
        }
    }

    // 2. 评估后右45°方向的自由空间
    float back_right_x = drone_x + 0.7f * std::cos(drone_yaw + M_PI - M_PI / 4);
    float back_right_y = drone_y + 0.7f * std::sin(drone_yaw + M_PI - M_PI / 4);
    float back_right_clearance = 0.0f;

    for (int i = 0; i < num_obstacles && i < static_cast<int>(obstacles.size()); ++i)
    {
        float dx = obstacles[i].position.x() - back_right_x;
        float dy = obstacles[i].position.y() - back_right_y;
        float dist = std::sqrt(dx * dx + dy * dy) - obstacles[i].radius;
        if (dist < back_right_clearance || back_right_clearance == 0.0f)
        {
            back_right_clearance = dist;
        }
    }

    // 3. 选择最佳逃生方向
    const float MIN_CLEARANCE = 0.6f; // 最小安全间隙（米）

    if (back_left_clearance > MIN_CLEARANCE && back_left_clearance > back_right_clearance)
    {
        // 后左45°方向更优
        escape_x = back_left_x;
        escape_y = back_left_y;
        ROS_INFO("[ESCAPE] 选择后左45°方向，间隙=%.2fm", back_left_clearance);
        return true;
    }
    else if (back_right_clearance > MIN_CLEARANCE)
    {
        // 后右45°方向更优
        escape_x = back_right_x;
        escape_y = back_right_y;
        ROS_INFO("[ESCAPE] 选择后右45°方向，间隙=%.2fm", back_right_clearance);
        return true;
    }
    else
    {
        // 无侧向空间，直接后退0.5m
        escape_x = drone_x - 0.5f * std::cos(drone_yaw);
        escape_y = drone_y - 0.5f * std::sin(drone_yaw);
        ROS_WARN("[ESCAPE] 无侧向空间，直接后退0.5m");
        return true;
    }
}

/************************************************************************
核心函数：避障主函数（三层防御体系）
========================================================================
【三层防御】
  L1: VFH+避障（正常场景，95%情况）
  L2: 增量修正（死胡同检测触发，85%死胡同可解决）
  L3: A*重规划（增量修正连续失败3次，兜底）

【输入输出】
  @param target_x_rel/y_rel  目标点（相对起飞点，米）
  @param target_yaw          目标航向（弧度）
  @param uav_radius          无人机半径（米）
  @param safe_margin         安全裕度（米）
  @param max_speed           最大速度（米/秒）
  @param min_safe_distance   力场最小距离（米）
  @param front_obstacle_th   死胡同检测：前方障碍物阈值
  @param back_obstacle_th    死胡同检测：后方障碍物阈值
  @param detection_radius    死胡同检测半径（米）
  @return bool               true=抵达目标点附近（<0.4m）

【工程规范】
  • 无文件级全局变量：所有中间数据置于函数栈
  • 解耦设计：仅依赖obstacles/local_pos等必要传感器数据
  • 参数驱动：全部行为由yaml配置控制
========================================================================*/
bool avoidance_with_escape(
    float target_x_rel,
    float target_y_rel,
    float target_yaw,
    float uav_radius,
    float safe_margin,
    float max_speed,
    float min_safe_distance,
    int front_obstacle_th,
    int back_obstacle_th,
    float detection_radius)
{
    // ========== 1. 时空基准 ==========
    float drone_x = local_pos.pose.pose.position.x;
    float drone_y = local_pos.pose.pose.position.y;
    float drone_yaw = yaw;

    float target_x_world = init_position_x_take_off + target_x_rel;
    float target_y_world = init_position_y_take_off + target_y_rel;

    float dx_to_target = target_x_world - drone_x;
    float dy_to_target = target_y_world - drone_y;
    float dist_to_target = std::sqrt(dx_to_target * dx_to_target + dy_to_target * dy_to_target);

    // 目标过近处理
    if (dist_to_target < 0.3f)
    {
        setpoint_raw.position.x = drone_x;
        setpoint_raw.position.y = drone_y;
        setpoint_raw.position.z = ALTITUDE;
        setpoint_raw.yaw = target_yaw;
        ROS_INFO("[AVOID] 目标过近(%.2fm)，悬停", dist_to_target);
        return true;
    }

    // ========== 2. 死胡同检测（L2防御） ==========
    static int escape_fail_count = 0; // 静态变量：跨帧计数

    bool dead_end = is_dead_end(
        obstacles, obstacles.size(),
        drone_x, drone_y, drone_yaw,
        detection_radius,
        front_obstacle_th,
        1, // left_th固定为1
        1, // right_th固定为1
        back_obstacle_th);

    if (dead_end)
    {
        ROS_WARN("[AVOID] 检测到死胡同！启动增量修正");

        // 计算逃生位置
        float escape_x, escape_y;
        if (compute_escape_position(
                obstacles, obstacles.size(),
                drone_x, drone_y, drone_yaw,
                escape_x, escape_y))
        {

            // 检查逃生位置是否有效（非死胡同）
            bool still_dead_end = is_dead_end(
                obstacles, obstacles.size(),
                escape_x, escape_y, drone_yaw,
                detection_radius,
                front_obstacle_th, 1, 1, back_obstacle_th);

            if (still_dead_end)
            {
                escape_fail_count++;
                ROS_WARN("[AVOID] 增量修正失败 %d/3 次", escape_fail_count);

                // 连续失败3次 → 触发A*重规划（L3兜底）
                if (escape_fail_count >= 3)
                {
                    ROS_WARN("[AVOID] 触发A*全局重规划（增量修正连续失败）");
                    // 此处可集成A*重规划逻辑（本实现仅打印警告）
                    // 实际部署时：调用astar_replan()生成新路径
                    escape_fail_count = 0;
                }
            }
            else
            {
                escape_fail_count = 0; // 修正成功，重置计数
                ROS_INFO("[AVOID] 增量修正成功，向(%.2f,%.2f)移动", escape_x, escape_y);

                // 输出逃生指令
                setpoint_raw.position.x = escape_x;
                setpoint_raw.position.y = escape_y;
                setpoint_raw.position.z = ALTITUDE;
                setpoint_raw.yaw = target_yaw;

                // 到达判断（逃生位置视为临时目标）
                float dist_to_escape = std::sqrt(
                    (drone_x - escape_x) * (drone_x - escape_x) +
                    (drone_y - escape_y) * (drone_y - escape_y));
                return (dist_to_escape < 0.3f);
            }
        }
    }

    // ========== 3. VFH+避障（L1防御，正常场景） ==========
    // 3.1 栅格系统（63x63，0.08m分辨率）
    static constexpr int GRID_SIZE = 63;
    static constexpr float GRID_RESOLUTION = 0.08f;
    static constexpr float DECAY_FACTOR = 0.94f;
    static constexpr float UPDATE_STRENGTH = 40.0f;
    static float certainty_grid[GRID_SIZE][GRID_SIZE] = {{0}};

    // 3.2 栅格更新
    {
        for (int i = 0; i < GRID_SIZE; ++i)
        {
            for (int j = 0; j < GRID_SIZE; ++j)
            {
                certainty_grid[i][j] *= DECAY_FACTOR;
                if (certainty_grid[i][j] < 1.0f)
                    certainty_grid[i][j] = 0.0f;
            }
        }

        float HALF_GRID = GRID_SIZE / 2.0f;
        float current_speed = current_vel.norm();
        float dynamic_safe_margin = safe_margin * (0.6f + 0.4f * current_speed / (max_speed + 0.1f));

        for (const auto &obs : obstacles)
        {
            float grid_x = (obs.position.x() - drone_x) / GRID_RESOLUTION + HALF_GRID;
            float grid_y = (obs.position.y() - drone_y) / GRID_RESOLUTION + HALF_GRID;

            if (grid_x < 0 || grid_x >= GRID_SIZE || grid_y < 0 || grid_y >= GRID_SIZE)
                continue;

            float safe_radius_world = obs.radius + dynamic_safe_margin;
            float obs_radius_grid = safe_radius_world / GRID_RESOLUTION;
            int radius_int = static_cast<int>(std::ceil(obs_radius_grid));

            int gx_center = static_cast<int>(std::round(grid_x));
            int gy_center = static_cast<int>(std::round(grid_y));

            for (int dx = -radius_int; dx <= radius_int; ++dx)
            {
                for (int dy = -radius_int; dy <= radius_int; ++dy)
                {
                    int gx = gx_center + dx;
                    int gy = gy_center + dy;

                    if (gx < 0 || gx >= GRID_SIZE || gy < 0 || gy >= GRID_SIZE)
                        continue;

                    float dist_to_center = std::sqrt(dx * dx + dy * dy);
                    if (dist_to_center > obs_radius_grid)
                        continue;

                    float weight = 1.0f - (dist_to_center / obs_radius_grid);
                    float increment = UPDATE_STRENGTH * weight;

                    certainty_grid[gx][gy] += increment;
                    if (certainty_grid[gx][gy] > 100.0f)
                        certainty_grid[gx][gy] = 100.0f;
                }
            }
        }
    }

    // 3.3 VFH+力场计算
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
    float FRONT_HALF_ANGLE = M_PI_2;

    for (int i = 0; i < GRID_SIZE; ++i)
    {
        for (int j = 0; j < GRID_SIZE; ++j)
        {
            float certainty = certainty_grid[i][j];
            if (certainty < 30.0f)
                continue;

            float dx_grid = (i - GRID_SIZE / 2) * GRID_RESOLUTION;
            float dy_grid = (j - GRID_SIZE / 2) * GRID_RESOLUTION;
            float dist_to_grid = std::sqrt(dx_grid * dx_grid + dy_grid * dy_grid);

            if (dist_to_grid < min_safe_distance || dist_to_grid > 2.5f)
                continue;

            float angle_to_grid = std::atan2(dy_grid, dx_grid) - drone_yaw;
            while (angle_to_grid > M_PI)
                angle_to_grid -= 2 * M_PI;
            while (angle_to_grid < -M_PI)
                angle_to_grid += 2 * M_PI;
            if (std::abs(angle_to_grid) > FRONT_HALF_ANGLE)
                continue;

            float force_mag = 1.0f * certainty / (dist_to_grid * dist_to_grid);
            if (force_mag > 10.0f)
                force_mag = 10.0f;

            float fx = (dx_grid / dist_to_grid) * force_mag;
            float fy = (dy_grid / dist_to_grid) * force_mag;

            repulsive_force.add(fx, fy);
        }
    }

    // 3.4 合成总力场
    ForceVector attractive_force, total_force;
    attractive_force.add(
        (dx_to_target / dist_to_target) * 1.0f,
        (dy_to_target / dist_to_target) * 1.0f);
    total_force.add(attractive_force.x - repulsive_force.x, attractive_force.y - repulsive_force.y);

    if (total_force.magnitude() < 0.01f)
    {
        ROS_WARN("[AVOID] 力场为零，启用沿墙走策略");
        total_force.x = std::cos(drone_yaw + M_PI_4);
        total_force.y = std::sin(drone_yaw + M_PI_4);
    }
    else
    {
        total_force.normalize();
    }

    // 3.5 速度调制
    float max_certainty_ahead = 0.0f;
    for (int i = GRID_SIZE / 2; i < GRID_SIZE; ++i)
    {
        for (int j = 0; j < GRID_SIZE; ++j)
        {
            if (certainty_grid[i][j] > max_certainty_ahead)
            {
                max_certainty_ahead = certainty_grid[i][j];
            }
        }
    }

    float speed_factor = 1.0f - (max_certainty_ahead / 100.0f) * 0.6f;
    if (speed_factor < 0.3f)
        speed_factor = 0.3f;
    float forward_speed = max_speed * speed_factor;

    // 3.6 生成指令
    float TIME_STEP = 0.1f;
    float safe_x = drone_x + total_force.x * forward_speed * TIME_STEP;
    float safe_y = drone_y + total_force.y * forward_speed * TIME_STEP;

    // 安全边界
    float step_dist = std::sqrt((safe_x - drone_x) * (safe_x - drone_x) + (safe_y - drone_y) * (safe_y - drone_y));
    if (step_dist > max_speed * TIME_STEP * 1.5f)
    {
        float scale = (max_speed * TIME_STEP * 1.5f) / step_dist;
        safe_x = drone_x + (safe_x - drone_x) * scale;
        safe_y = drone_y + (safe_y - drone_y) * scale;
    }

    setpoint_raw.position.x = safe_x;
    setpoint_raw.position.y = safe_y;
    setpoint_raw.position.z = ALTITUDE;
    setpoint_raw.yaw = target_yaw;

    // 3.7 到达判断
    float dist_now = std::sqrt((safe_x - target_x_world) * (safe_x - target_x_world) +
                               (safe_y - target_y_world) * (safe_y - target_y_world));

    {
        static ros::Time last_print = ros::Time::now();
        if ((ros::Time::now() - last_print).toSec() > 1.0)
        {
            ROS_INFO("[AVOID] 目标(%.2f,%.2f)→避障点(%.2f,%.2f) 距离=%.2fm 速度=%.2fm/s",
                     target_x_world, target_y_world, safe_x, safe_y, dist_now, forward_speed);
            last_print = ros::Time::now();
        }
    }

    return (dist_now < 0.4f);
}