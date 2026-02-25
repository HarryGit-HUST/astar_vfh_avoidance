#ifndef ASTAR_H
#define ASTAR_H

#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h> // 地图消息
#include <visualization_msgs/MarkerArray.h>
#include <livox_ros_driver/CustomMsg.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <eigen3/Eigen/Dense>
#include <vector>
#include <cmath>
#include <algorithm>
#include <queue>

#include "new_detect_obs.h"

using namespace std;

// 全局变量声明
extern mavros_msgs::PositionTarget setpoint_raw;
extern nav_msgs::Odometry local_pos;
extern double current_yaw;

// 状态记录
extern float init_pos_x, init_pos_y, init_pos_z;
extern bool flag_init_pos;

// ============================================================================
// 1. 动态栅格地图 (支持记忆与衰减)
// ============================================================================
struct OccupancyGrid2D
{
    static const int GRID_W = 200;
    static const int GRID_H = 200;

    // 使用 int 而不是 uint8，方便计算衰减，取值范围 [0, 100]
    int cells[GRID_W][GRID_H];

    float resolution;
    float origin_x;
    float origin_y;

    OccupancyGrid2D();

    bool world_to_grid(float wx, float wy, int &gx, int &gy) const;
    void grid_to_world(int gx, int gy, float &wx, float &wy) const;
    bool is_occupied(int gx, int gy) const;

    // 核心修改：带衰减的更新
    void update_with_decay(const std::vector<Obstacle> &obstacles, float drone_r, float safe_margin);
};

// ============================================================================
// 2. B-Spline 平滑
// ============================================================================
class BSplinePlanner
{
public:
    static std::vector<Eigen::Vector2f> generate_smooth_path(const std::vector<Eigen::Vector2f> &control_points, int points_per_segment);
};

// ============================================================================
// 3. 核心算法函数
// ============================================================================

// 注意：传入的是私有句柄 private_nh
void load_parameters(ros::NodeHandle &private_nh);

void state_cb(const mavros_msgs::State::ConstPtr &msg);
void local_pos_cb(const nav_msgs::Odometry::ConstPtr &msg);

void pub_viz_astar_path(const std::vector<Eigen::Vector2f> &path);
void pub_viz_smooth_path(const std::vector<Eigen::Vector2f> &path);
void pub_viz_vfh_vectors(float target_yaw, float selected_yaw, const Eigen::Vector2f &pos);
void pub_viz_grid_map(const OccupancyGrid2D &grid);

bool run_astar(const OccupancyGrid2D &grid, Eigen::Vector2f start, Eigen::Vector2f goal, std::vector<Eigen::Vector2f> &out_path);

bool is_path_blocked(const std::vector<Eigen::Vector2f> &path, const OccupancyGrid2D &grid, float check_radius);

Eigen::Vector2f get_lookahead_point(const std::vector<Eigen::Vector2f> &path, Eigen::Vector2f curr_pos, float lookahead_dist);

bool run_vfh_plus(Eigen::Vector2f target, const std::vector<Obstacle> &obs, bool &need_replan);

#endif // ASTAR_H