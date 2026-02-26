/*******************************************************************************
 * @file astar.h
 * @brief 无人机智能避障规划系统 - 记忆增强版
 * @details 集成三区衰减模型与超级血条机制，解决拓扑震荡问题
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
#include <nav_msgs/OccupancyGrid.h>
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

#include "new_detect_obs.h"

using namespace std;

// ============================================================================
// 全局变量声明
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
 * @brief 记忆型动态栅格地图
 * @note 使用 int 存储置信度，上限 1000 (超级血条)
 */
struct OccupancyGrid2D
{
    static const int GRID_W = 200;
    static const int GRID_H = 200;

    // 升级为 int，支持 [0, 1000] 的范围
    int cells[GRID_W][GRID_H];

    float resolution;
    float origin_x;
    float origin_y;

    // 参数：超级血条上限
    const int MAX_HEALTH = 1000;
    // 参数：A* 判定障碍物的阈值 (50/1000 = 5% 概率即视为障碍)
    const int OBS_THRESHOLD = 50;

    OccupancyGrid2D();

    bool world_to_grid(float wx, float wy, int &gx, int &gy) const;
    void grid_to_world(int gx, int gy, float &wx, float &wy) const;
    bool is_occupied(int gx, int gy) const;

    // 核心：三区衰减更新
    void update_with_memory(const std::vector<Obstacle> &obstacles, float drone_r, float safe_margin);
};

/**
 * @brief B-Spline 轨迹平滑器
 */
class BSplinePlanner
{
public:
    static std::vector<Eigen::Vector2f> generate_smooth_path(const std::vector<Eigen::Vector2f> &control_points, int points_per_segment);
};

// ============================================================================
// 核心算法函数声明
// ============================================================================

void load_parameters(ros::NodeHandle &nh);
void state_cb(const mavros_msgs::State::ConstPtr &msg);
void local_pos_cb(const nav_msgs::Odometry::ConstPtr &msg);

// 可视化接口
void pub_viz_astar_path(const std::vector<Eigen::Vector2f> &path);
void pub_viz_smooth_path(const std::vector<Eigen::Vector2f> &path);
void pub_viz_vfh_vectors(float target_yaw, float selected_yaw, const Eigen::Vector2f &pos);
void pub_viz_grid_map(const OccupancyGrid2D &grid);

// A* 规划
bool run_astar(const OccupancyGrid2D &grid, Eigen::Vector2f start, Eigen::Vector2f goal, std::vector<Eigen::Vector2f> &out_path);

// 路径有效性检查
bool is_path_blocked(const std::vector<Eigen::Vector2f> &path, const OccupancyGrid2D &grid, float check_radius);

// 获取前视点
Eigen::Vector2f get_lookahead_point(const std::vector<Eigen::Vector2f> &path, Eigen::Vector2f curr_pos, float lookahead_dist);

// VFH+ 局部规划
bool run_vfh_plus(Eigen::Vector2f target, const std::vector<Obstacle> &obs, bool &need_replan);

#endif // ASTAR_H