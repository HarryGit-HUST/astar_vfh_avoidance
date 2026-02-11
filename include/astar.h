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

#define ALTITUDE 0.7f

mavros_msgs::PositionTarget setpoint_raw;

Eigen::Vector2f current_pos;
Eigen::Vector2f current_vel;

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
    current_pos = Eigen::Vector2f(local_pos.pose.pose.position.x, local_pos.pose.pose.position.y);

    tf::Vector3 body_vel(local_pos.twist.twist.linear.x, local_pos.twist.twist.linear.y, local_pos.twist.twist.linear.z);
    tf::Matrix3x3 rot_matrix(quat);
    tf::Vector3 world_vel = rot_matrix * body_vel;
    current_vel = Eigen::Vector2f(world_vel.x(), world_vel.y());

    if (flag_init_position == false && (local_pos.pose.pose.position.z > 0.1))
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
float mission_cruise_timeout = 180.0f;
ros::Time mission_cruise_start_time;
bool mission_cruise_timeout_flag = false;
bool mission_pos_cruise_flag = false;
bool mission_pos_cruise(float x, float y, float z, float target_yaw, float error_max);
bool mission_pos_cruise(float x, float y, float z, float target_yaw, float error_max)
{
    if (mission_pos_cruise_flag == false)
    {
        mission_pos_cruise_last_position_x = local_pos.pose.pose.position.x;
        mission_pos_cruise_last_position_y = local_pos.pose.pose.position.y;
        mission_pos_cruise_flag = true;
        mission_cruise_start_time = ros::Time::now();
        mission_cruise_timeout_flag = false;
    }
    ros::Duration elapsed_time = ros::Time::now() - mission_cruise_start_time;
    if (elapsed_time.toSec() > mission_cruise_timeout && !mission_cruise_timeout_flag)
    {
        ROS_WARN("[巡航超时] 已耗时%.1f秒（阈值%.1f秒），强制切换下一个任务！", elapsed_time.toSec(), mission_cruise_timeout);
        mission_cruise_timeout_flag = true;
        mission_pos_cruise_flag = false;
        return true;
    }
    setpoint_raw.type_mask = /*1 + 2 + 4 */ +8 + 16 + 32 + 64 + 128 + 256 + 512 /*+ 1024 */ + 2048;
    setpoint_raw.coordinate_frame = 1;
    setpoint_raw.position.x = x + init_position_x_take_off;
    setpoint_raw.position.y = y + init_position_y_take_off;
    setpoint_raw.position.z = z + init_position_z_take_off;
    setpoint_raw.yaw = target_yaw;
    if (fabs(local_pos.pose.pose.position.x - x - init_position_x_take_off) < error_max && fabs(local_pos.pose.pose.position.y - y - init_position_y_take_off) < error_max && fabs(local_pos.pose.pose.position.z - z - init_position_z_take_off) < error_max && fabs(yaw - target_yaw) < 0.1)
    {
        ROS_INFO("到达目标点，巡航点任务完成");
        mission_cruise_timeout_flag = false;
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
        precision_land_init_position_flag = false;
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

// ========== 修复2: 全英文变量名（移除所有中文） ==========
struct OccupancyGrid2D
{
    uint8_t cells[100][100];
    float resolution;
    float origin_x;
    float origin_y;

    OccupancyGrid2D()
    {
        resolution = 0.1f;
        origin_x = -5.0f;
        origin_y = -5.0f;
        for (int i = 0; i < 100; ++i)
            for (int j = 0; j < 100; ++j)
                cells[i][j] = 0;
    }

    bool world_to_grid(float wx, float wy, int &gx, int &gy) const
    {
        gx = static_cast<int>((wx - origin_x) / resolution);
        gy = static_cast<int>((wy - origin_y) / resolution);
        return (gx >= 0 && gx < 100 && gy >= 0 && gy < 100);
    }

    void grid_to_world(int gx, int gy, float &wx, float &wy) const
    {
        wx = origin_x + gx * resolution;
        wy = origin_y + gy * resolution;
    }

    void update_with_obstacles(
        const std::vector<Obstacle> &obstacles,
        float drone_radius,
        float safety_margin)
    {
        // ========== 修复3: 英文变量名 ==========
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

        // ========== 关键修复: 全英文变量名 ==========
        float expansion_radius = drone_radius + safety_margin;                            // 原"膨胀半径"
        int expansion_cells = static_cast<int>(std::ceil(expansion_radius / resolution)); // 原"膨胀栅格"

        for (const auto &obs : obstacles)
        {
            int gx, gy;
            if (world_to_grid(obs.position.x(), obs.position.y(), gx, gy))
            {
                cells[gx][gy] = 100;

                for (int dx = -expansion_cells; dx <= expansion_cells; ++dx)
                { // 原"膨胀栅格"
                    for (int dy = -expansion_cells; dy <= expansion_cells; ++dy)
                    { // 原"膨胀栅格"
                        int nx = gx + dx;
                        int ny = gy + dy;
                        if (nx >= 0 && nx < 100 && ny >= 0 && ny < 100)
                        {
                            float dist = std::sqrt(dx * dx + dy * dy);
                            if (dist <= expansion_cells)
                            {                                                                              // 原"膨胀栅格"
                                uint8_t val = static_cast<uint8_t>(100 * (1.0f - dist / expansion_cells)); // 原"膨胀栅格"
                                if (val > cells[nx][ny])
                                {
                                    cells[nx][ny] = val;
                                }
                            }
                        }
                    }
                }
            }
        }
    }
};

// ========== A*规划器（完整英文实现，无中文） ==========
int astar_plan(
    const OccupancyGrid2D &grid,
    float start_x, float start_y,
    float goal_x, float goal_y,
    float *path_x, float *path_y,
    int max_points)
{
    int start_gx, start_gy, goal_gx, goal_gy;
    if (!grid.world_to_grid(start_x, start_y, start_gx, start_gy) ||
        !grid.world_to_grid(goal_x, goal_y, goal_gx, goal_gy))
    {
        ROS_ERROR("[A*] 起点/目标点超出地图范围");
        return 0;
    }

    if (grid.cells[start_gx][start_gy] > 50)
    {
        ROS_WARN("[A*] 起点在障碍物内，尝试偏移");
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
                    {
                        start_gx = nx;
                        start_gy = ny;
                        found = true;
                        ROS_INFO("[A*] 起点偏移至(%d,%d)", nx, ny);
                    }
                }
            }
        }
        if (!found)
        {
            ROS_ERROR("[A*] 无法找到有效起点");
            return 0;
        }
    }

    if (grid.cells[goal_gx][goal_gy] > 50)
    {
        ROS_WARN("[A*] 目标点在障碍物内，尝试偏移");
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
                        ROS_INFO("[A*] 目标点偏移至(%d,%d)", nx, ny);
                    }
                }
            }
        }
        if (!found)
        {
            ROS_ERROR("[A*] 无法找到有效目标点");
            return 0;
        }
    }

    auto cmp = [](const std::pair<int, std::pair<int, int>> &a,
                  const std::pair<int, std::pair<int, int>> &b)
    {
        return a.first > b.first;
    };
    std::priority_queue<std::pair<int, std::pair<int, int>>,
                        std::vector<std::pair<int, std::pair<int, int>>>,
                        decltype(cmp)>
        open_set(cmp);

    std::unordered_map<int, int> g_score;
    std::unordered_map<int, std::pair<int, int>> parent_map;
    std::unordered_set<int> closed_set;

    open_set.push({std::abs(goal_gx - start_gx) + std::abs(goal_gy - start_gy), {start_gx, start_gy}});
    g_score[start_gx * 1000 + start_gy] = 0;

    std::pair<int, int> goal_node = {-1, -1};
    int iterations = 0;
    const int MAX_ITERATIONS = 10000;

    while (!open_set.empty() && iterations < MAX_ITERATIONS)
    {
        auto current = open_set.top();
        open_set.pop();
        int cx = current.second.first;
        int cy = current.second.second;
        int current_key = cx * 1000 + cy;

        if (cx == goal_gx && cy == goal_gy)
        {
            goal_node = {cx, cy};
            break;
        }

        if (closed_set.count(current_key))
            continue;
        closed_set.insert(current_key);

        const int dx[4] = {1, -1, 0, 0};
        const int dy[4] = {0, 0, 1, -1};

        for (int i = 0; i < 4; ++i)
        {
            int nx = cx + dx[i];
            int ny = cy + dy[i];

            if (nx < 0 || nx >= 100 || ny < 0 || ny >= 100 || grid.cells[nx][ny] > 50)
                continue;

            int neighbor_key = nx * 1000 + ny;
            if (closed_set.count(neighbor_key))
                continue;

            int tentative_g = g_score[current_key] + 1;

            if (!g_score.count(neighbor_key) || tentative_g < g_score[neighbor_key])
            {
                g_score[neighbor_key] = tentative_g;
                int h = std::abs(goal_gx - nx) + std::abs(goal_gy - ny);
                open_set.push({tentative_g + h, {nx, ny}});
                parent_map[neighbor_key] = {cx, cy};
            }
        }

        iterations++;
    }

    int path_size = 0;
    if (goal_node.first != -1)
    {
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

        for (size_t i = 0; i < grid_path.size() && path_size < max_points; ++i)
        {
            float wx, wy;
            grid.grid_to_world(grid_path[i].first, grid_path[i].second, wx, wy);
            path_x[path_size] = wx - init_position_x_take_off;
            path_y[path_size] = wy - init_position_y_take_off;
            path_size++;
        }

        ROS_INFO("[A*] 规划成功，生成 %d 个路径点", path_size);
    }
    else
    {
        ROS_ERROR("[A*] 规划失败！无可行路径（迭代=%d）", iterations);
    }

    return path_size;
}

// ========== 走廊生成器（全英文） ==========
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
    if (num_points < 2 || max_size < 2)
    {
        ROS_ERROR("[CORRIDOR] 路径点不足或输出数组过小");
        return 0;
    }
    if (base_width < 0.5f || base_width > 3.0f)
    {
        ROS_WARN("[CORRIDOR] 走廊宽度%.2fm超出合理范围[0.5,3.0]，强制钳位", base_width);
        base_width = std::max(0.5f, std::min(3.0f, base_width));
    }

    const int MAX_SMOOTH_POINTS = 200;
    float smooth_x[MAX_SMOOTH_POINTS];
    float smooth_y[MAX_SMOOTH_POINTS];
    int smooth_count = 0;

    float last_x = astar_path_x[0] + init_position_x_take_off;
    float last_y = astar_path_y[0] + init_position_y_take_off;
    smooth_x[smooth_count] = last_x;
    smooth_y[smooth_count] = last_y;
    smooth_count++;

    for (int i = 1; i < num_points && smooth_count < MAX_SMOOTH_POINTS - 1; ++i)
    {
        float curr_x = astar_path_x[i] + init_position_x_take_off;
        float curr_y = astar_path_y[i] + init_position_y_take_off;

        float dx = curr_x - last_x;
        float dy = curr_y - last_y;
        float dist = std::sqrt(dx * dx + dy * dy);

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

    if (smooth_count < MAX_SMOOTH_POINTS)
    {
        smooth_x[smooth_count] = astar_path_x[num_points - 1] + init_position_x_take_off;
        smooth_y[smooth_count] = astar_path_y[num_points - 1] + init_position_y_take_off;
        smooth_count++;
    }

    ROS_DEBUG("[CORRIDOR] 路径平滑: %d点 → %d点", num_points, smooth_count);

    int corridor_count = 0;
    const float CURVATURE_THRESHOLD = 0.4f;

    for (int i = 0; i < smooth_count && corridor_count < max_size; ++i)
    {
        float width = base_width;

        if (i > 0 && i < smooth_count - 1)
        {
            float prev_x = smooth_x[i - 1], prev_y = smooth_y[i - 1];
            float curr_x = smooth_x[i], curr_y = smooth_y[i];
            float next_x = smooth_x[i + 1], next_y = smooth_y[i + 1];

            float v1x = curr_x - prev_x, v1y = curr_y - prev_y;
            float v2x = next_x - curr_x, v2y = next_y - curr_y;

            float cross = v1x * v2y - v1y * v2x;
            float v1_len = std::sqrt(v1x * v1x + v1y * v1y);
            float v2_len = std::sqrt(v2x * v2x + v2y * v2y);
            float curvature = std::abs(cross) / (std::pow(v1_len * v2_len, 1.5f) + 1e-6f);

            static float curvature_history[5] = {0};
            static int history_idx = 0;
            curvature_history[history_idx] = curvature;
            history_idx = (history_idx + 1) % 5;

            float avg_curvature = 0;
            for (int j = 0; j < 5; ++j)
                avg_curvature += curvature_history[j];
            avg_curvature /= 5.0f;

            if (avg_curvature > CURVATURE_THRESHOLD)
            {
                width *= 1.2f;
                ROS_DEBUG("[CORRIDOR] 急弯检测: idx=%d 曲率=%.2f → 宽度=%.2fm",
                          i, avg_curvature, width);
            }
        }

        corridor_x[corridor_count] = smooth_x[i];
        corridor_y[corridor_count] = smooth_y[i];
        corridor_width[corridor_count] = width;
        corridor_count++;
    }

    ROS_INFO("[CORRIDOR] 生成 %d 个走廊点，基础宽度=%.2fm", corridor_count, base_width);
    return corridor_count;
}

// ========== VFH+避障（全英文，含走廊软约束） ==========
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
    out_need_replan = false;

    float drone_x = local_pos.pose.pose.position.x;
    float drone_y = local_pos.pose.pose.position.y;
    float drone_yaw = yaw;

    float target_x_world = init_position_x_take_off + target_x_rel;
    float target_y_world = init_position_y_take_off + target_y_rel;

    float dx_to_target = target_x_world - drone_x;
    float dy_to_target = target_y_world - drone_y;
    float dist_to_target = std::sqrt(dx_to_target * dx_to_target + dy_to_target * dy_to_target);

    if (dist_to_target < 0.3f)
    {
        setpoint_raw.position.x = drone_x;
        setpoint_raw.position.y = drone_y;
        setpoint_raw.position.z = ALTITUDE;
        setpoint_raw.yaw = target_yaw;
        ROS_INFO("[VFH+] 目标过近(%.2fm)，悬停", dist_to_target);
        return true;
    }

    static constexpr int GRID_SIZE = 63;
    static constexpr float GRID_RESOLUTION = 0.08f;
    static constexpr float DECAY_FACTOR = 0.94f;
    static constexpr float UPDATE_STRENGTH = 40.0f;
    static float certainty_grid[GRID_SIZE][GRID_SIZE] = {{0}};

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

    static constexpr int HISTOGRAM_BINS = 72;
    float histogram[HISTOGRAM_BINS] = {0};

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

            float world_angle = std::atan2(dy_grid, dx_grid);
            float relative_angle = world_angle - drone_yaw;
            while (relative_angle > M_PI)
                relative_angle -= 2 * M_PI;
            while (relative_angle < -M_PI)
                relative_angle += 2 * M_PI;

            int bin = static_cast<int>(
                std::floor((relative_angle + M_PI) / (2 * M_PI) * HISTOGRAM_BINS));
            if (bin < 0)
                bin = 0;
            if (bin >= HISTOGRAM_BINS)
                bin = HISTOGRAM_BINS - 1;

            float weight = certainty / (dist_to_grid * dist_to_grid);
            histogram[bin] += weight;
        }
    }

    float smoothed_histogram[HISTOGRAM_BINS] = {0};
    int smooth_radius = 2;

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

    // ========== 异常状态检测（内嵌） ==========
    static std::vector<std::pair<float, float>> pos_history;
    static std::vector<float> yaw_history;
    static bool is_oscillating = false;
    static int oscillation_frames = 0;

    pos_history.push_back({drone_x, drone_y});
    yaw_history.push_back(drone_yaw);
    if (pos_history.size() > 60)
    {
        pos_history.erase(pos_history.begin());
        yaw_history.erase(yaw_history.begin());
    }

    if (!is_oscillating && pos_history.size() >= 30)
    {
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

        float min_yaw = *std::min_element(yaw_history.begin(), yaw_history.end());
        float max_yaw = *std::max_element(yaw_history.begin(), yaw_history.end());
        float yaw_range = std::abs(max_yaw - min_yaw) * 180.0f / M_PI;

        if (pos_std < 0.25f && yaw_range > 90.0f)
        {
            is_oscillating = true;
            oscillation_frames = 40;
            ROS_WARN("[VFH+] 检测到振荡！位置标准差=%.2fm 角度变化=%.0f°",
                     pos_std, yaw_range);
            out_need_replan = true;
        }
    }

    if (is_oscillating)
    {
        oscillation_frames--;
        if (oscillation_frames <= 0)
        {
            is_oscillating = false;
            ROS_INFO("[VFH+] 振荡恢复完成");
        }
    }

    static int target_blocked_count = 0;
    bool target_blocked = false;

    float ray_steps = std::max(5.0f, dist_to_target / 0.2f);
    for (int s = 1; s <= static_cast<int>(ray_steps); ++s)
    {
        float ratio = static_cast<float>(s) / ray_steps;
        float check_x = drone_x + dx_to_target * ratio;
        float check_y = drone_y + dy_to_target * ratio;

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
        target_blocked_count = std::max(0, target_blocked_count - 1);
    }

    static std::vector<Obstacle> known_obstacles;
    bool new_large_obstacle = false;

    for (const auto &obs : obstacles)
    {
        if (obs.radius > 0.8f)
        {
            bool known = false;
            for (const auto &known_obs : known_obstacles)
            {
                float dx = obs.position.x() - known_obs.position.x();
                float dy = obs.position.y() - known_obs.position.y();
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
        ROS_INFO("[VFH+] 视野变化！检测到新大型障碍物");
        known_obstacles = obstacles;
        out_need_replan = true;
    }

    // ========== 振荡恢复策略 ==========
    if (is_oscillating && oscillation_frames > 0)
    {
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

        float escape_angle = drone_yaw;
        if (nearest_obs_dist < std::numeric_limits<float>::max())
        {
            float tangent_x = -(nearest_obs.y() - drone_y) / nearest_obs_dist;
            float tangent_y = (nearest_obs.x() - drone_x) / nearest_obs_dist;

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
            escape_angle = std::atan2(dy_to_target, dx_to_target);
        }

        float recovery_speed = max_speed * 0.4f;
        float TIME_STEP = 0.1f;
        float safe_x = drone_x + std::cos(escape_angle) * recovery_speed * TIME_STEP;
        float safe_y = drone_y + std::sin(escape_angle) * recovery_speed * TIME_STEP;

        setpoint_raw.position.x = safe_x;
        setpoint_raw.position.y = safe_y;
        setpoint_raw.position.z = ALTITUDE;
        setpoint_raw.yaw = target_yaw;

        ROS_WARN("[VFH+] 振荡恢复中 (%d帧剩余) → 切线方向%.1f°",
                 oscillation_frames, escape_angle * 180.0f / M_PI);

        float dist_now = std::sqrt((safe_x - target_x_world) * (safe_x - target_x_world) +
                                   (safe_y - target_y_world) * (safe_y - target_y_world));
        return (dist_now < 0.4f);
    }

    // ========== VFH+基础避障 ==========
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

    // ========== 走廊软约束融合 ==========
    ForceVector corridor_attraction;
    float dist_to_corridor_center = std::numeric_limits<float>::max();
    bool in_corridor = false;

    if (enable_corridor && corridor_size > 0)
    {
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

        dist_to_corridor_center = min_dist;
        float half_width = corridor_width[nearest_idx] / 2.0f;

        if (dist_to_corridor_center < half_width)
        {
            in_corridor = true;

            float to_center_x = corridor_x[nearest_idx] - drone_x;
            float to_center_y = corridor_y[nearest_idx] - drone_y;
            float to_center_mag = std::sqrt(to_center_x * to_center_x + to_center_y * to_center_y);

            if (to_center_mag > 1e-6f)
            {
                to_center_x /= to_center_mag;
                to_center_y /= to_center_mag;

                float weight = 0.3f + 0.7f * (1.0f - dist_to_corridor_center / half_width);
                float attraction_mag = 0.8f * dist_to_corridor_center * weight;

                corridor_attraction.x = to_center_x * attraction_mag;
                corridor_attraction.y = to_center_y * attraction_mag;
            }
        }
    }

    // ========== 历史方向记忆 + 滞后效应 ==========
    static std::vector<int> bin_history;
    static int prev_selected_bin = -1;

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

    if (prev_selected_bin >= 0)
    {
        bin_history.push_back(prev_selected_bin);
        if (bin_history.size() > 3)
            bin_history.erase(bin_history.begin());
    }

    std::vector<int> candidates;
    float prev_selected_cost = std::numeric_limits<float>::max();

    for (int bin = 0; bin < HISTOGRAM_BINS; ++bin)
    {
        if (smoothed_histogram[bin] > 60.0f)
            continue;

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

    int best_bin = candidates[0];
    float min_cost = std::numeric_limits<float>::max();
    for (int bin : candidates)
    {
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

    // ========== 动态权重融合 ==========
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
        min_obstacle_dist = 5.0f;

    float vfh_weight = 1.0f - std::min(1.0f, min_obstacle_dist / 1.5f);
    vfh_weight = std::max(0.3f, vfh_weight);
    float corridor_weight = 1.0f - vfh_weight;

    ForceVector attractive_force;
    attractive_force.add(
        (dx_to_target / dist_to_target) * 1.0f,
        (dy_to_target / dist_to_target) * 1.0f);

    ForceVector total_force;
    total_force.add(
        attractive_force.x - repulsive_force.x + corridor_attraction.x * corridor_weight,
        attractive_force.y - repulsive_force.y + corridor_attraction.y * corridor_weight);

    if (total_force.magnitude() < 0.01f)
    {
        ROS_WARN("[VFH+] 力场为零，启用沿墙走策略");
        total_force.x = std::cos(drone_yaw + M_PI_4);
        total_force.y = std::sin(drone_yaw + M_PI_4);
    }
    else
    {
        total_force.normalize();
    }

    // ========== 速度调制 + 指令生成 ==========
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

    float speed_factor = 1.0f - (forward_congestion / 60.0f) * 0.6f;
    if (speed_factor < 0.3f)
        speed_factor = 0.3f;
    float forward_speed = max_speed * speed_factor;

    float TIME_STEP = 0.1f;
    float safe_x = drone_x + total_force.x * forward_speed * TIME_STEP;
    float safe_y = drone_y + total_force.y * forward_speed * TIME_STEP;

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

    // ========== 到达判断 + 调试输出 ==========
    float dist_now = std::sqrt((safe_x - target_x_world) * (safe_x - target_x_world) +
                               (safe_y - target_y_world) * (safe_y - target_y_world));

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