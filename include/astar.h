/*******************************************************************************
astar.h - 无人机智能避障系统核心头文件
******************************************************************************/
#ifndef ASTAR_H
#define ASTAR_H

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
#include <queue>
#include <unordered_map>
#include <unordered_set>

using namespace std;

// ============================================================================
// 全局常量定义
// ============================================================================
#define ALTITUDE 0.7f

// ============================================================================
// 全局变量声明 ( extern ) - 定义在 astar.cpp 中
// ============================================================================
extern mavros_msgs::PositionTarget setpoint_raw;
extern Eigen::Vector2f current_pos;
extern Eigen::Vector2f current_vel;

extern mavros_msgs::State mavros_connection_state;

extern tf::Quaternion quat;
extern nav_msgs::Odometry local_pos;
extern double roll, pitch, yaw;

extern float init_position_x_take_off;
extern float init_position_y_take_off;
extern float init_position_z_take_off;
extern float init_yaw_take_off;
extern bool flag_init_position;

extern float mission_pos_cruise_last_position_x;
extern float mission_pos_cruise_last_position_y;
extern float mission_cruise_timeout;
extern ros::Time mission_cruise_start_time;
extern bool mission_cruise_timeout_flag;
extern bool mission_pos_cruise_flag;

extern float precision_land_init_position_x;
extern float precision_land_init_position_y;
extern bool precision_land_init_position_flag;
extern bool hovor_done;
extern bool land_done;
extern ros::Time precision_land_last_time;

// ============================================================================
// 函数声明
// ============================================================================
void state_cb(const mavros_msgs::State::ConstPtr &msg);
void local_pos_cb(const nav_msgs::Odometry::ConstPtr &msg);

bool mission_pos_cruise(float x, float y, float z, float target_yaw, float error_max);
bool precision_land(float err_max);


// ============================================================================
// 数据结构
// ============================================================================
struct OccupancyGrid2D
{
    uint8_t cells[100][100];
    float resolution;
    float origin_x;
    float origin_y;

    OccupancyGrid2D();
    bool world_to_grid(float wx, float wy, int &gx, int &gy) const;
    void grid_to_world(int gx, int gy, float &wx, float &wy) const;
    void update_with_obstacles(const std::vector<Obstacle> &obstacles, float drone_radius, float safety_margin);
};

// ============================================================================
// 算法函数声明
// ============================================================================
int astar_plan(const OccupancyGrid2D &grid, float start_x, float start_y, float goal_x, float goal_y, float *path_x, float *path_y, int max_points);
int incremental_astar_plan(const OccupancyGrid2D &grid, float start_x, float start_y, float goal_x, float goal_y, float *path_x, float *path_y, int max_points);
int generate_corridor(const float *astar_path_x, const float *astar_path_y, int num_points, float base_width, float *corridor_x, float *corridor_y, float *corridor_width, int max_size);
bool vfh_plus_with_corridor(float target_x_rel, float target_y_rel, float target_yaw, float uav_radius, float safe_margin, float max_speed, float min_safe_distance, const float *corridor_x, const float *corridor_y, const float *corridor_width, int corridor_size, bool enable_corridor, bool &out_need_replan);

#endif // ASTAR_H