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
函数 :计算速度
*************************************************************************/

// 定义状态机
enum class AvoidState
{
    IDLE,         // 直飞目标
    OBS_DETECTED, // 发现障碍，准备减速/规划
    AVOIDING,     // 正在飞向切点
    RECOVERING    // 越过切点，正在回归航线（带保护期）
};

// 静态变量保持状态 (也可以封装进类)
static AvoidState g_avoid_state = AvoidState::IDLE;
static ros::Time g_state_enter_time;
static Eigen::Vector2f g_latched_tangent; // 锁定的切点（可选，或每帧更新）

/**
 * @brief 计算碰撞锥相关参数（完全保留原有逻辑）
 */
std::tuple<float, float> calculateCollisionCone(
    const Eigen::Vector2f &UAV_pos,
    const Eigen::Vector2f &UAV_dir,
    const ObsRound &obs,
    float UAV_radius)
{
    Eigen::Vector2f r = obs.position - UAV_pos;
    float r_norm = r.norm();
    if (r_norm < 1e-3)
    {
        return {M_PI, M_PI}; // 距离过近，默认最大锥角
    }

    // 碰撞锥开度角：基于安全半径计算
    float cone_opening_angle = asin(std::min(1.0f, obs.safe_radius / r_norm));
    float dir_angle = acos(std::max(-1.0f, std::min(1.0f, UAV_dir.dot(r.normalized()))));

    return {cone_opening_angle, dir_angle};
} /**
   * @brief 选择最优切点（核心修改：选y坐标相对于无人机最远的切点，多障碍直接选全局最远）
   */
Eigen::Vector2f selectOptimalTangent(
    const std::vector<ObsRound> &obs_rounds,
    const Eigen::Vector2f &UAV_pos,
    const Eigen::Vector2f &target)
{
    // 无障碍物：返回目标点
    if (obs_rounds.empty())
        return target;

    // 提取无人机当前y坐标（核心参考值）
    float uav_y = UAV_pos.y();

    // 单障碍物：选该障碍左/右切点中y坐标最远的（|y_tangent - uav_y|最大）
    if (obs_rounds.size() == 1)
    {
        const auto &obs = obs_rounds[0];
        // 计算左/右切点y与无人机y的绝对值差
        float left_y_diff = fabs(obs.left_point.y() - uav_y);
        float right_y_diff = fabs(obs.right_point.y() - uav_y);

        // 选y差值更大的切点
        Eigen::Vector2f selected_tangent = (left_y_diff > right_y_diff) ? obs.left_point : obs.right_point;

        ROS_INFO("单障碍切点选择：左切点y=%.2f（差=%.2f），右切点y=%.2f（差=%.2f），选y最远切点=(%.2f,%.2f)",
                 obs.left_point.y(), left_y_diff, obs.right_point.y(), right_y_diff,
                 selected_tangent.x(), selected_tangent.y());
        return selected_tangent;
    }

    // 多障碍物：直接选全局y坐标最远的切点（|y_tangent - uav_y|最大）
    Eigen::Vector2f farthest_tangent; // 全局y最远切点
    float max_y_diff = -1.0f;         // 最大y差值（初始为负，确保首次赋值）

    for (const auto &obs : obs_rounds)
    {
        // 步骤1：给当前障碍选左/右切点中y坐标最远的
        float left_y_diff = fabs(obs.left_point.y() - uav_y);
        float right_y_diff = fabs(obs.right_point.y() - uav_y);
        Eigen::Vector2f obs_farthest = (left_y_diff > right_y_diff) ? obs.left_point : obs.right_point;
        float obs_y_diff = (left_y_diff > right_y_diff) ? left_y_diff : right_y_diff;

        // 步骤2：对比当前障碍的y最远切点与全局，保留更远的
        if (obs_y_diff > max_y_diff)
        {
            max_y_diff = obs_y_diff;
            farthest_tangent = obs_farthest;
        }

        // 日志：输出当前障碍的切点y信息
        ROS_INFO("障碍切点：左(%.2f, %.2f)y差=%.2f | 右(%.2f, %.2f)y差=%.2f | 选该障碍y最远切点=(%.2f,%.2f)",
                 obs.left_point.x(), obs.left_point.y(), left_y_diff,
                 obs.right_point.x(), obs.right_point.y(), right_y_diff,
                 obs_farthest.x(), obs_farthest.y());
    }

    // 边界处理：若所有y差值为0（极端情况），返回目标点
    if (max_y_diff < 1e-3)
    {
        ROS_WARN("所有切点y坐标与无人机几乎重合，返回目标点");
        return target;
    }

    ROS_INFO("多障碍全局y最远切点（y差=%.2f）：(%.2f,%.2f)", max_y_diff, farthest_tangent.x(), farthest_tangent.y());
    return farthest_tangent;
}
/**
 * @brief 筛选激活障碍物（碰撞锥重叠的障碍物）
 */
std::vector<ObsRound> getActiveObs(
    const std::vector<ObsRound> &obs_rounds,
    const Eigen::Vector2f &UAV_pos,
    const Eigen::Vector2f &target)
{
    std::vector<ObsRound> active_obs;
    Eigen::Vector2f UAV_dir = target - UAV_pos;
    if (UAV_dir.norm() < 1e-3)
        return active_obs;
    UAV_dir.normalize();

    for (const auto &obs : obs_rounds)
    {
        auto [cone_opening, dir_angle] = calculateCollisionCone(UAV_pos, UAV_dir, obs, UAV_radius);
        // 碰撞锥重叠（方向角≤开度角+5°安全裕度）→ 激活
        if (dir_angle <= cone_opening + 0.087f)
        { // 0.087rad=5°
            active_obs.push_back(obs);
        }
    }

    // 按距离排序（近→远），解决多障碍物优先级
    std::sort(active_obs.begin(), active_obs.end(),
              [&UAV_pos](const ObsRound &a, const ObsRound &b)
              {
                  return (a.position - UAV_pos).norm() < (b.position - UAV_pos).norm();
              });
    return active_obs;
}

/**
 * @brief 判断障碍物是否处于“碰撞锥”内（即是否阻挡了通往目标的直线路径）
 * @param obs 障碍物结构体
 * @param uav_pos 无人机当前位置
 * @param target 目标位置
 * @return true 存在碰撞风险，需规避；false 路径净空
 */
bool isPointInCollisionCone(const ObsRound &obs,
                            const Eigen::Vector2f &uav_pos,
                            const Eigen::Vector2f &target)
{
    // 1. 计算向量
    Eigen::Vector2f uav_to_target = target - uav_pos;
    Eigen::Vector2f uav_to_obs = obs.position - uav_pos;

    float dist_to_target = uav_to_target.norm();
    if (dist_to_target < 0.1f)
        return false; // 已到达目标

    // 2. 投影计算：判断障碍物是否在无人机的前方
    // 使用点积求 obs 在 uav_to_target 方向上的投影长度
    float projection = uav_to_obs.dot(uav_to_target.normalized());

    // 逻辑判定：
    // a) projection < 0: 障碍物在无人机身后 -> 无威胁
    // b) projection > dist_to_target: 障碍物在目标点后面 -> 无威胁
    if (projection < 0 || projection > dist_to_target)
    {
        return false;
    }

    // 3. 计算垂直距离：障碍物圆心距离“飞行直线”的最近偏移量
    // 利用勾股定理：dist^2 = projection^2 + vertical_dist^2
    float dist_sq = uav_to_obs.squaredNorm();
    float vertical_dist_sq = dist_sq - (projection * projection);

    // 为了稳健，防止浮点数微小误差导致负数
    float vertical_dist = std::sqrt(std::max(0.0f, vertical_dist_sq));

    // 4. 碰撞判定：如果垂直距离小于安全半径，则在锥体内
    return (vertical_dist < obs.safe_radius);
}

/**
 * @brief 基于速度控制的切线避障函数
 * @param target 世界坐标系目标点
 * @param uav_pos 无人机当前绝对位置
 * @param current_vel 无人机当前实际速度向量 (Eigen::Vector2f)
 * @return Eigen::Vector2f 期望速度向量 (Target Velocity)
 */

// --- 参数配置 (建议从YAML读取) ---
float MAX_SPEED = 0.5f;      // 巡航速度
float AVOID_SPEED = 0.4f;    // 规避时的过弯速度
float MIN_SPEED = 0.15f;     // 减速阶段的最低速度
float DETECT_DIST = 4.0f;    // 减速触发距离
float RECOVERY_LATCH = 1.0f; // 状态锁存时间

Eigen::Vector2f coneAvoidanceByVelocity(
    const Eigen::Vector2f &target,
    const Eigen::Vector2f &uav_pos,
    const Eigen::Vector2f &current_vel)
{
    // 1. 获取当前最具威胁的障碍物 (基于直线路径探测)
    ObsRound *threat = nullptr;
    float min_dist = 999.0f;
    for (auto &obs : obs_rounds)
    {
        if (isPointInCollisionCone(obs, uav_pos, target))
        {
            float d = (obs.position - uav_pos).norm();
            if (d < min_dist)
            {
                min_dist = d;
                threat = &obs;
            }
        }
    }

    Eigen::Vector2f target_vel(0, 0);

    switch (g_avoid_state)
    {
    case AvoidState::IDLE:
        if (threat != nullptr)
        {
            g_avoid_state = AvoidState::OBS_DETECTED;
            g_state_enter_time = ros::Time::now();
        }
        // 目标方向速度
        target_vel = (target - uav_pos).normalized() * MAX_SPEED;
        break;

    case AvoidState::OBS_DETECTED:
        if (threat == nullptr)
        {
            g_avoid_state = AvoidState::IDLE;
            target_vel = (target - uav_pos).normalized() * MAX_SPEED;
        }
        else
        {
            // --- 减速逻辑：距离越近，速度越慢 ---
            float speed_factor = std::clamp((min_dist - 1.5f) / (DETECT_DIST - 1.5f), 0.0f, 1.0f);
            float desired_speed = MIN_SPEED + (MAX_SPEED - MIN_SPEED) * speed_factor;

            target_vel = (target - uav_pos).normalized() * desired_speed;

            // 当速度降到预设规避速度以下，或者距离障碍物已近（2.5m），切入规避
            if (current_vel.norm() < AVOID_SPEED + 0.05f || min_dist < 2.5f)
            {
                g_avoid_state = AvoidState::AVOIDING;
                ROS_WARN("State -> AVOIDING (Velocity Control)");
            }
        }
        break;

    case AvoidState::AVOIDING:
        if (threat == nullptr)
        {
            g_avoid_state = AvoidState::RECOVERING;
            g_state_enter_time = ros::Time::now();
        }
        else
        {
            // --- 切线速度逻辑 ---
            // 选择离目标更近的那个切点方向
            float dist_l = (threat->left_point - target).norm();
            float dist_r = (threat->right_point - target).norm();
            Eigen::Vector2f best_pt = (dist_l < dist_r) ? threat->left_point : threat->right_point;

            // 计算从当前位置指向切点的速度向量
            target_vel = (best_pt - uav_pos).normalized() * AVOID_SPEED;

            // 辅助逻辑：如果无人机已经越过了障碍物（即障碍物在身后）
            Eigen::Vector2f uav_to_obs = threat->position - uav_pos;
            Eigen::Vector2f uav_to_target = target - uav_pos;
            // 计算夹角的余弦值
            float cos_theta = uav_to_obs.dot(uav_to_target) / (uav_to_obs.norm() * uav_to_target.norm());

            // cos(90度)=0, cos(110度)≈-0.34
            // 只有当障碍物明显处于侧后方（角度 > 105度）时才退出
            if (cos_theta < -0.25f)
            {
                g_avoid_state = AvoidState::RECOVERING;
                g_state_enter_time = ros::Time::now();
            }
        }
        break;

    case AvoidState::RECOVERING:
        // 恢复期：指向目标，但限制速度，防止猛冲
        target_vel = (target - uav_pos).normalized() * AVOID_SPEED;

        if ((ros::Time::now() - g_state_enter_time).toSec() > RECOVERY_LATCH)
        {
            if (threat == nullptr)
            {
                g_avoid_state = AvoidState::IDLE;
                ROS_INFO("State -> IDLE (Path Clear)");
            }
            else
            {
                g_avoid_state = AvoidState::AVOIDING;
            }
        }
        break;
    }

    return target_vel;
}

bool cone_avoidance_movement(float target_x, float target_y, float target_z,
                             float target_yaw, float UAV_radius,
                             float time_final, float err_max)
{
    // ================= 1. 初始化（首次调用） =================
    static bool is_init = false;
    static ros::Time start_time;
    if (!is_init)
    {
        start_time = ros::Time::now();
        is_init = true;
        g_avoid_state = AvoidState::IDLE; // 重置避障状态机
        ROS_INFO("圆锥避障任务启动，超时阈值：%.1f秒", time_final);
    }

    // ================= 2. 终止条件判断（保留原有逻辑） =================
    ros::Duration elapsed_time = ros::Time::now() - start_time;
    // 2.1 超时终止
    if (elapsed_time.toSec() >= time_final)
    {
        ROS_WARN("圆锥避障任务超时（已耗时%.1f秒），准备降落！", elapsed_time.toSec());
        is_init = false;
        return true;
    }
    // 2.2 到达目标点终止
    float abs_target_x = target_x + init_position_x_take_off;
    float abs_target_y = target_y + init_position_y_take_off;
    float dist_x = local_pos.pose.pose.position.x - abs_target_x;
    float dist_y = local_pos.pose.pose.position.y - abs_target_y;
    float dist_xy = hypotf(dist_x, dist_y);
    float dist_z = fabs(local_pos.pose.pose.position.z - (target_z + init_position_z_take_off));
    if (dist_xy < err_max && dist_z < err_max)
    {
        ROS_INFO("到达目标点（距离误差：%.3f米），准备降落！", dist_xy);
        is_init = false;
        return true;
    }

    // ================= 3. 黑箱函数调用：转换障碍物为扩增圆+切点 =================
    extern std::vector<Obstacle> obstacles; // 原有障碍物列表
    transObs(obstacles);                    // 调用黑箱函数，生成obs_rounds（含切点/安全半径）
    static int log_count = 0;
    if (if_debug == 1 && ++log_count % 10 == 0)
    {
        for (int i = 0; i < obs_rounds.size(); i++)
        {

            ROS_INFO("障碍物%d:(%.2f,%.2f),r=%.2f,sr=%.2f,l(%.2f,%.2f),r(%.2f,%.2f)", i, obs_rounds[i].position.x(), obs_rounds[i].position.y(), obs_rounds[i].radius, obs_rounds[i].safe_radius, obs_rounds[i].left_point.x(), obs_rounds[i].left_point.y(), obs_rounds[i].right_point.x(), obs_rounds[i].right_point.y());
        }
    }

    // ================= 4. 输入参数准备 =================
    Eigen::Vector2f UAV_pos = current_pos;              // 无人机当前位置
    Eigen::Vector2f UAV_vel = current_vel;              // 无人机当前速度
    Eigen::Vector2f target(abs_target_x, abs_target_y); // 绝对目标点

    // ================= 5. 调用圆锥避障位置计算函数 =================
    Eigen::Vector2f vel_cmd = coneAvoidanceByVelocity(target, UAV_pos, UAV_vel);

    float vel_combination = hypot(vel_cmd.x(), vel_cmd.y());
    if (vel_combination > MAX_SPEED)
    {
        vel_cmd.x() *= MAX_SPEED / vel_combination;
        vel_cmd.y() *= MAX_SPEED / vel_combination;
    }

    // ================= 6. 设置位置控制指令（保留原有逻辑，仅标注建议） =================
    // 【优化建议】type_mask=8+16+32+64+128+256+1024 更合理（移除2048，避免位置指令失效）
    setpoint_raw.type_mask = 1 + 2 /*+ 4 + 8 + 16 */ + 32 + 64 + 128 + 256 + 1024 + 2048;
    setpoint_raw.coordinate_frame = 1;                             // 局部NED坐标系
    setpoint_raw.velocity.x = vel_cmd.x();                         // 避障计算的下一个位置X
    setpoint_raw.velocity.y = vel_cmd.y();                         // 避障计算的下一个位置Y
    setpoint_raw.position.z = target_z + init_position_z_take_off; // 固定Z高度
    setpoint_raw.yaw = target_yaw;                                 // 固定偏航角

    // ================= 7. 日志输出（增强避障状态） =================
    std::string state_str;
    switch (g_avoid_state)
    {
    case AvoidState::IDLE:
        state_str = "正常飞行";
        break;
    case AvoidState::OBS_DETECTED:
        state_str = "减速";
        break;
    case AvoidState::AVOIDING:
        state_str = "飞向切点";
        break;
    case AvoidState::RECOVERING:
        state_str = "回归目标";
        break;
    }
    ROS_INFO(
        "状态：%s | 当前位置：(%.2f, %.2f) | 目标点：(%.2f, %.2f) | 当前速度：(%.2f, %.2f) | 剩余时间：%.1f秒",
        state_str.c_str(),
        UAV_pos.x(), UAV_pos.y(),
        target.x(), target.y(),
        vel_cmd.x(), vel_cmd.y(),
        time_final - elapsed_time.toSec());

    // ================= 8. 未满足终止条件，继续执行 =================
    return false;
}