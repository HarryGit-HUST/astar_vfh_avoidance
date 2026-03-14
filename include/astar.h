#ifndef ASTAR_H
#define ASTAR_H

#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
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
#include <tf/transform_datatypes.h>
#include <eigen3/Eigen/Dense>
#include <vector>
#include <cmath>
#include <algorithm>
#include <queue>
#include <string>

// 依赖库
#include <std_msgs/Bool.h>
#include <std_msgs/Int8.h>
#include <std_msgs/String.h>

#include "new_detect_obs.h"
#include "ring_crossing.h"

using namespace std;

extern mavros_msgs::PositionTarget setpoint_raw;
extern nav_msgs::Odometry local_pos;
extern double current_yaw;

extern float init_pos_x, init_pos_y, init_pos_z;
extern bool flag_init_pos;

extern std::string takeoff_color;
extern std::string land_color;
extern bool land_detected;
extern geometry_msgs::PointStamped yolo_result;

// ============================================================================
// 修复后的动态栅格地图类
// ============================================================================
struct OccupancyGrid2D
{
    int grid_w;
    int grid_h;
    std::vector<std::vector<int>> cells; // 真正的动态二维数组
    float resolution;
    float origin_x;
    float origin_y;
    const int MAX_HEALTH = 1000;
    const int OBS_THRESHOLD = 50;

    OccupancyGrid2D();
    void init(float res, float w_m, float h_m, float ox, float oy);
    bool world_to_grid(float wx, float wy, int &gx, int &gy) const;
    void grid_to_world(int gx, int gy, float &wx, float &wy) const;
    bool is_occupied(int gx, int gy) const;
    void update_with_memory(const std::vector<Obstacle> &static_walls, float drone_r, float safe_margin);
    void clear();
    
};


class BSplinePlanner
{
public:
    static std::vector<Eigen::Vector2f> generate_smooth_path(const std::vector<Eigen::Vector2f> &control_points, int points_per_segment);
};

void load_parameters(ros::NodeHandle &nh);
void state_cb(const mavros_msgs::State::ConstPtr &msg);
void local_pos_cb(const nav_msgs::Odometry::ConstPtr &msg);

void yolo_result_cb(const geometry_msgs::PointStamped::ConstPtr &msg);
void takeoff_cb(const std_msgs::String::ConstPtr &msg);
void land_color_cb(const std_msgs::String::ConstPtr &msg);
void land_detected_cb(const std_msgs::Bool::ConstPtr &msg);

void pub_viz_astar_path(const std::vector<Eigen::Vector2f> &path);
void pub_viz_smooth_path(const std::vector<Eigen::Vector2f> &path);
void pub_viz_vfh_vectors(float target_yaw, float selected_yaw, const Eigen::Vector2f &pos, float hist[72]);
void pub_viz_grid_map(const OccupancyGrid2D &grid);

bool run_astar(const OccupancyGrid2D &grid, Eigen::Vector2f start, Eigen::Vector2f goal, std::vector<Eigen::Vector2f> &out_path);
bool is_path_blocked(const std::vector<Eigen::Vector2f> &path, const OccupancyGrid2D &grid, float check_radius);
Eigen::Vector2f get_lookahead_point(const std::vector<Eigen::Vector2f> &path, Eigen::Vector2f curr_pos, float lookahead_dist);
bool run_vfh_plus(Eigen::Vector2f target, const std::vector<Obstacle> &obs, bool &need_replan);

float satfunc(float data, float Max);
float calc_smooth_yaw(float target_yaw, float current_yaw, float dt);

#endif // ASTAR_H