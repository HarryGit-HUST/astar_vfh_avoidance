/*******************************************************************************
astar.h - 修复与优化版
******************************************************************************/
#ifndef ASTAR_H
#define ASTAR_H

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/CommandLong.h>
#include <nav_msgs/Odometry.h>
#include <livox_ros_driver/CustomMsg.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <eigen3/Eigen/Dense>
#include <vector>
#include <cmath>
#include <algorithm>
#include <queue>
#include <unordered_map>
#include <unordered_set>

// 引入你的感知头文件
#include "new_detect_obs.h"

using namespace std;

#define ALTITUDE 0.7f

// ============================================================================
// 全局变量声明 (与 astar.cpp 对应)
// ============================================================================
extern mavros_msgs::PositionTarget setpoint_raw;
extern Eigen::Vector2f current_pos;
extern Eigen::Vector2f current_vel;
extern mavros_msgs::State mavros_connection_state;
extern nav_msgs::Odometry local_pos;
extern double roll, pitch, yaw;

// 状态标志位
extern float init_position_x_take_off;
extern float init_position_y_take_off;
extern float init_position_z_take_off;
extern bool flag_init_position;

// 任务相关
extern int mission_num;
extern float target_x;
extern float target_y;
extern float UAV_radius;

// ============================================================================
// 数据结构定义
// ============================================================================
struct OccupancyGrid2D
{
    uint8_t cells[200][200]; // 扩大一点地图范围以防越界，原100可能偏小
    float resolution;
    float origin_x;
    float origin_y;

    OccupancyGrid2D();
    bool world_to_grid(float wx, float wy, int &gx, int &gy) const;
    void grid_to_world(int gx, int gy, float &wx, float &wy) const;

    // 优化：增加边界检查
    bool is_occupied(int gx, int gy) const;
    void update_with_obstacles(const std::vector<Obstacle> &obstacles, float drone_radius, float safety_margin);
};

// ============================================================================
// 函数声明
// ============================================================================
void state_cb(const mavros_msgs::State::ConstPtr &msg);
void local_pos_cb(const nav_msgs::Odometry::ConstPtr &msg);

// 核心算法
int astar_plan(const OccupancyGrid2D &grid, float start_x, float start_y, float goal_x, float goal_y, float *path_x, float *path_y, int max_points);
int generate_corridor(const float *astar_path_x, const float *astar_path_y, int num_points, float base_width, float *corridor_x, float *corridor_y, float *corridor_width, int max_size);
bool vfh_plus_with_corridor(float target_x_rel, float target_y_rel, float target_yaw, float uav_radius, float safe_margin, float max_speed, float min_safe_distance, const float *corridor_x, const float *corridor_y, const float *corridor_width, int corridor_size, bool enable_corridor, bool &out_need_replan);

// 任务控制
bool mission_pos_cruise(float x, float y, float z, float target_yaw, float error_max);
bool precision_land(float err_max);

#endif // ASTAR_H