/*******************************************************************************
 * @file astar.h
 * @brief 无人机智能避障规划系统核心头文件 (工程架构版)
 * @details 包含：
 *          1. OccupancyGrid2D: 动态栅格地图
 *          2. BSplinePlanner: B样条轨迹平滑
 *          3. GlobalPlanner: 加权 A* 算法
 *          4. LocalPlanner: VFH+ 局部避障与路径跟踪
 *          5. Visualization: RViz 调试接口
 ******************************************************************************/
#ifndef ASTAR_H
#define ASTAR_H

#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/MarkerArray.h>
#include <livox_ros_driver/CustomMsg.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <eigen3/Eigen/Dense>
#include <vector>
#include <cmath>
#include <algorithm>
#include <queue>
#include <limits>

// 引入感知模块 (Obstacle 结构定义)
#include "new_detect_obs.h"

using namespace std;

// ============================================================================
// 全局变量声明 (与 new_detect_obs.h 兼容)
// ============================================================================
extern mavros_msgs::PositionTarget setpoint_raw;
extern nav_msgs::Odometry local_pos;
extern double current_yaw;

// 状态记录
extern float init_pos_x, init_pos_y, init_pos_z;
extern bool flag_init_pos;

// ============================================================================
// 工具类定义
// ============================================================================

/**
 * @brief 动态栅格地图类
 * 负责将感知的几何障碍物映射为用于 A* 的栅格数据
 */
struct OccupancyGrid2D
{
    // 地图参数：20x20米范围，分辨率0.1m -> 200x200网格
    static const int GRID_W = 200;
    static const int GRID_H = 200;

    uint8_t cells[GRID_W][GRID_H];
    float resolution;
    float origin_x;
    float origin_y;

    OccupancyGrid2D();

    // 坐标转换与边界检查
    bool world_to_grid(float wx, float wy, int &gx, int &gy) const;
    void grid_to_world(int gx, int gy, float &wx, float &wy) const;
    bool is_occupied(int gx, int gy) const;

    // 障碍物更新 (严密圆形膨胀)
    void update_with_obstacles(const std::vector<Obstacle> &obstacles, float drone_r, float safe_margin);
};

/**
 * @brief B-Spline 轨迹平滑器
 */
class BSplinePlanner
{
public:
    // 根据控制点生成 3阶均匀 B 样条
    // points_per_segment: 插值密度
    static std::vector<Eigen::Vector2f> generate_smooth_path(const std::vector<Eigen::Vector2f> &control_points, int points_per_segment);
};

// ============================================================================
// 核心算法函数声明
// ============================================================================

// 参数加载
void load_parameters(ros::NodeHandle &nh);

// ROS 回调
void state_cb(const mavros_msgs::State::ConstPtr &msg);
void local_pos_cb(const nav_msgs::Odometry::ConstPtr &msg);

// 可视化接口
void pub_viz_astar_path(const std::vector<Eigen::Vector2f> &path);
void pub_viz_smooth_path(const std::vector<Eigen::Vector2f> &path); // 原走廊可视化升级为平滑轨迹
void pub_viz_vfh_vectors(float target_yaw, float selected_yaw, const Eigen::Vector2f &pos);

// A* 规划 (返回路径点列表)
bool run_astar(const OccupancyGrid2D &grid, Eigen::Vector2f start, Eigen::Vector2f goal, std::vector<Eigen::Vector2f> &out_path);

// 路径有效性检查 (增量更新核心)
// 返回 true 表示路径被新障碍物阻挡
bool is_path_blocked(const std::vector<Eigen::Vector2f> &path, const std::vector<Obstacle> &obs, float check_radius);

// 获取前视点 (Lookahead Point)
Eigen::Vector2f get_lookahead_point(const std::vector<Eigen::Vector2f> &path, Eigen::Vector2f curr_pos, float lookahead_dist);

// VFH+ 局部规划
// target: 局部目标点 (Lookahead Point)
// need_replan: 输出参数，若陷入死锁则置为 true
bool run_vfh_plus(Eigen::Vector2f target, const std::vector<Obstacle> &obs, bool &need_replan);

#endif // ASTAR_H