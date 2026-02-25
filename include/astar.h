/*******************************************************************************
 * astar.h - 无人机避障规划系统核心头文件 (工程严密版)
 * 包含：A*规划、VFH+局部避障、走廊生成、路径有效性检测、可视化接口
 ******************************************************************************/
#ifndef ASTAR_H
#define ASTAR_H

#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>                  // 用于发布轨迹
#include <visualization_msgs/MarkerArray.h> // 用于发布走廊和向量
#include <livox_ros_driver/CustomMsg.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <eigen3/Eigen/Dense>
#include <vector>
#include <cmath>
#include <algorithm>
#include <queue>

// 引入感知模块
#include "new_detect_obs.h"

using namespace std;

// ============================================================================
// 全局变量声明
// ============================================================================
extern mavros_msgs::PositionTarget setpoint_raw;
extern mavros_msgs::State mavros_connection_state;
extern nav_msgs::Odometry local_pos;
extern double current_yaw; // 统一变量名为 current_yaw

// 状态记录
extern float init_pos_x, init_pos_y, init_pos_z;
extern bool flag_init_pos;

// 任务控制
extern int mission_step;

// ============================================================================
// 数据结构定义
// ============================================================================
struct OccupancyGrid2D
{
    // 20x20m 地图，0.1m 分辨率 -> 200x200 栅格
    static const int GRID_W = 200;
    static const int GRID_H = 200;

    uint8_t cells[GRID_W][GRID_H];
    float resolution;
    float origin_x;
    float origin_y;

    OccupancyGrid2D();

    // 坐标转换
    bool world_to_grid(float wx, float wy, int &gx, int &gy) const;
    void grid_to_world(int gx, int gy, float &wx, float &wy) const;
    bool is_occupied(int gx, int gy) const;

    // 障碍物更新 (严密圆形膨胀)
    void update_with_obstacles(const std::vector<Obstacle> &obstacles, float drone_r, float safe_margin);
};

// ============================================================================
// 函数声明
// ============================================================================

// ROS 回调
void state_cb(const mavros_msgs::State::ConstPtr &msg);
void local_pos_cb(const nav_msgs::Odometry::ConstPtr &msg);

// 参数加载
void load_parameters(ros::NodeHandle &nh);

// 可视化工具 (新增)
void pub_viz_astar_path(const float *x, const float *y, int num);
void pub_viz_corridor(const float *x, const float *y, const float *w, int num);
void pub_viz_vfh_vectors(float target_yaw, float selected_yaw, const Eigen::Vector2f &pos);

// 核心算法
// 1. A* 规划
int astar_plan(const OccupancyGrid2D &grid, float start_x, float start_y, float goal_x, float goal_y, float *path_x, float *path_y, int max_points);

// 2. 走廊生成
int generate_corridor(const float *astar_x, const float *astar_y, int num, float base_w, float *corr_x, float *corr_y, float *corr_w, int max_size);

// 3. 路径有效性检查 (解决 VFH 与 走廊冲突的核心)
// 返回 true 表示当前路径被障碍物截断，需要重规划
bool check_global_path_blocked(const float *path_x, const float *path_y, int path_size, const std::vector<Obstacle> &obs, float check_radius);

// 4. VFH+ 局部避障
bool vfh_plus_with_corridor(float target_x_rel, float target_y_rel, float target_yaw,
                            const float *corr_x, const float *corr_y, const float *corr_w, int corr_size,
                            bool &out_need_replan);

// 任务控制
bool precision_land();

#endif // ASTAR_H