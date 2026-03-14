/**
 * @file astar.cpp
 * @brief 终极优化版：点云直入、平滑转向、参数外置、视觉之字形搜索
 */
#include "astar.h"
#include "ring_crossing.h"
#include <algorithm>
#include <clocale>
#include <cmath>
#include <iostream>

// ============================================================================
// 1. 全局变量
// ============================================================================
float target_x = 0.0f;
float target_y = 0.0f;
float if_debug = 1.0f;

float init_position_x_take_off = 0;
float init_position_y_take_off = 0;
float init_position_z_take_off = 0;
float init_yaw_take_off = 0;
bool flag_init_position = false;

ros::Time precision_land_last_time;
bool land_done = false;

// ROI点云与锁定机头
pcl::PointCloud<pcl::PointXY>::Ptr current_cloud(new pcl::PointCloud<pcl::PointXY>);
float current_target_yaw = 0.0f;

std::vector<Obstacle> obstacles;
std::vector<Obstacle> static_walls;
RingCrossing ring_ctrl;

int mission_step = 0;
mavros_msgs::PositionTarget setpoint_raw;
mavros_msgs::State mavros_connection_state;
nav_msgs::Odometry local_pos;
double current_yaw = 0.0;
double current_yaw_rate = 0.0;
tf::Quaternion quat;

float init_pos_x = 0, init_pos_y = 0, init_pos_z = 0;
bool flag_init_pos = false;

std::vector<Eigen::Vector2f> global_path_raw;
std::vector<Eigen::Vector2f> global_path_smooth;
ros::Time last_replan_time;
bool has_global_plan = false;

OccupancyGrid2D global_grid;

// Scan_Land
std::string takeoff_color = "";
std::string land_color = "";
bool land_detected = false;
geometry_msgs::PointStamped yolo_result;
std_msgs::Int8 mission_num_msg;

// [修复的降落全局变量]
bool region_scan_reset_requested = false;

float last_vfh_yaw = 0.0;
bool vfh_first_run = true;

// ============================================================================
// 2. 配置参数 (对应 YAML)
// ============================================================================
struct Config
{
    float takeoff_height;
    float uav_radius;
    float safe_margin;
    float max_speed;
    float min_safe_dist;
    float lookahead_dist;
    float astar_weight;
    float replan_cooldown;

    float check_radius_buffer;
    float wall_radius;
    int map_decay_rate;
    float rotation_gating_threshold;
    float max_yaw_rate;
    float yaw_smooth_weight;

    std::vector<float> wp1;
    std::vector<float> wp2;
    std::vector<float> wp3;
    std::vector<float> wp4;

    float err_max;
    float p_xy;
    float vel_track_max;
    float time_threshold;
    float yolo_follow_kp;

    float map_resolution;
    float map_width_m;
    float map_height_m;
    float map_origin_x;
    float map_origin_y;
} cfg;

ros::Publisher pub_setpoint;
ros::Publisher pub_viz_path_raw;
ros::Publisher pub_viz_path_smooth;
ros::Publisher pub_viz_vfh;
ros::Publisher pub_viz_map;
ros::Publisher mission_num_pub;

void yolo_result_cb(const geometry_msgs::PointStamped::ConstPtr &msg) { yolo_result = *msg; }
void takeoff_cb(const std_msgs::String::ConstPtr &msg) { takeoff_color = msg->data; }
void land_color_cb(const std_msgs::String::ConstPtr &msg) { land_color = msg->data; }
void land_detected_cb(const std_msgs::Bool::ConstPtr &msg) { land_detected = msg->data; }

void load_parameters(ros::NodeHandle &nh)
{
    nh.param<float>("target_x", target_x, 6.0f);
    nh.param<float>("target_y", target_y, 0.0f);
    nh.param<float>("debug_mode", if_debug, 1.0f);

    nh.param<float>("planner/takeoff_height", cfg.takeoff_height, 1.2f);
    nh.param<float>("planner/uav_radius", cfg.uav_radius, 0.3f);
    nh.param<float>("planner/safe_margin", cfg.safe_margin, 0.3f);
    nh.param<float>("planner/max_speed", cfg.max_speed, 0.6f);
    nh.param<float>("planner/lookahead_dist", cfg.lookahead_dist, 1.5f);
    nh.param<float>("planner/astar_weight", cfg.astar_weight, 1.5f);
    nh.param<float>("planner/replan_cooldown", cfg.replan_cooldown, 1.0f);
    nh.param<int>("planner/map_decay_rate", cfg.map_decay_rate, 2);
    nh.param<float>("planner/check_radius_buffer", cfg.check_radius_buffer, 0.1f);
    nh.param<float>("planner/wall_radius", cfg.wall_radius, 0.2f);
    nh.param<float>("planner/rotation_gating_threshold", cfg.rotation_gating_threshold, 0.3f);
    nh.param<float>("planner/max_yaw_rate", cfg.max_yaw_rate, 0.5f);
    nh.param<float>("planner/yaw_smooth_weight", cfg.yaw_smooth_weight, 0.1f);
    nh.param<float>("planner/min_safe_dist", cfg.min_safe_dist, 0.4f);

    nh.param<std::vector<float>>("planner/wp1", cfg.wp1, {0.0, 3.5});
    nh.param<std::vector<float>>("planner/wp2", cfg.wp2, {3.5, 3.5});
    nh.param<std::vector<float>>("planner/wp3", cfg.wp3, {3.5, 0.0});
    nh.param<std::vector<float>>("planner/wp4", cfg.wp4, {6.0, 0.0});

    nh.param<float>("scan_land/err_max", cfg.err_max, 0.3f);
    nh.param<float>("scan_land/p_xy", cfg.p_xy, 0.3f);
    nh.param<float>("scan_land/vel_track_max", cfg.vel_track_max, 0.4f);
    nh.param<float>("scan_land/time_threshold", cfg.time_threshold, 3.0f);
    nh.param<float>("scan_land/yolo_follow_kp", cfg.yolo_follow_kp, -2.0f);

    nh.param<float>("planner/map_resolution", cfg.map_resolution, 0.1f);
    nh.param<float>("planner/map_width_m", cfg.map_width_m, 20.0f);
    nh.param<float>("planner/map_height_m", cfg.map_height_m, 20.0f);
    nh.param<float>("planner/map_origin_x", cfg.map_origin_x, -10.0f);
    nh.param<float>("planner/map_origin_y", cfg.map_origin_y, -10.0f);

    ROS_INFO("=== 参数加载完成 ===");
}

float satfunc(float data, float Max)
{
    if (std::abs(data) > Max)
        return (data > 0) ? Max : -Max;
    return data;
}

float get_dist(float tx, float ty)
{
    float dx = (init_pos_x + tx) - local_pos.pose.pose.position.x;
    float dy = (init_pos_y + ty) - local_pos.pose.pose.position.y;
    return std::hypot(dx, dy);
}

float get_yaw_diff(float target_yaw)
{
    float diff = target_yaw - current_yaw;
    while (diff > M_PI)
        diff -= 2 * M_PI;
    while (diff < -M_PI)
        diff += 2 * M_PI;
    return std::abs(diff);
}

float calc_smooth_yaw(float target_yaw, float current_yaw, float dt)
{
    float diff = target_yaw - current_yaw;
    while (diff > M_PI)
        diff -= 2 * M_PI;
    while (diff < -M_PI)
        diff += 2 * M_PI;
    float max_step = cfg.max_yaw_rate * dt;
    if (std::abs(diff) < max_step)
        return target_yaw;
    else
        return current_yaw + (diff > 0 ? max_step : -max_step);
}

float dist_sq_point_to_segment(const Eigen::Vector2f &p, const Eigen::Vector2f &s_start, const Eigen::Vector2f &s_end)
{
    Eigen::Vector2f v = s_end - s_start;
    Eigen::Vector2f w = p - s_start;
    float c1 = w.dot(v);
    if (c1 <= 0)
        return w.squaredNorm();
    float c2 = v.dot(v);
    if (c2 <= c1)
        return (p - s_end).squaredNorm();
    float b = c1 / c2;
    Eigen::Vector2f pb = s_start + b * v;
    return (p - pb).squaredNorm();
}

// ---------------------------------------------------------
// 2D 计算几何工具集
// ---------------------------------------------------------
float cross2d(const Eigen::Vector2f &O, const Eigen::Vector2f &A, const Eigen::Vector2f &B)
{
    return (A.x() - O.x()) * (B.y() - O.y()) - (A.y() - O.y()) * (B.x() - O.x());
}

std::vector<Eigen::Vector2f> getConvexHull(std::vector<Eigen::Vector2f> pts)
{
    if (pts.size() <= 2)
        return pts;
    std::sort(pts.begin(), pts.end(), [](const Eigen::Vector2f &a, const Eigen::Vector2f &b)
              { return a.x() < b.x() || (std::abs(a.x() - b.x()) < 1e-5 && a.y() < b.y()); });
    std::vector<Eigen::Vector2f> hull;
    for (const auto &p : pts)
    {
        while (hull.size() >= 2 && cross2d(hull[hull.size() - 2], hull.back(), p) <= 0)
            hull.pop_back();
        hull.push_back(p);
    }
    size_t lower_size = hull.size();
    for (int i = pts.size() - 2; i >= 0; --i)
    {
        while (hull.size() > lower_size && cross2d(hull[hull.size() - 2], hull.back(), pts[i]) <= 0)
            hull.pop_back();
        hull.push_back(pts[i]);
    }
    if (hull.size() > 1)
        hull.pop_back();
    return hull;
}

float distToPolygon(const Eigen::Vector2f &pt, const std::vector<Eigen::Vector2f> &poly)
{
    if (poly.empty())
        return 1e9;
    bool inside = true;
    float min_dist_sq = 1e9;
    int n = poly.size();
    for (int i = 0; i < n; ++i)
    {
        Eigen::Vector2f p1 = poly[i];
        Eigen::Vector2f p2 = poly[(i + 1) % n];
        if (cross2d(p1, p2, pt) < 0)
            inside = false;
        float d_sq = dist_sq_point_to_segment(pt, p1, p2);
        if (d_sq < min_dist_sq)
            min_dist_sq = d_sq;
    }
    return inside ? 0.0f : std::sqrt(min_dist_sq);
}

void build_static_walls()
{
    if (!static_walls.empty())
        return;

    float front_x = init_pos_x + 5.3f;
    float back_x = init_pos_x - 0.5f;
    float left_y = init_pos_y + 1.0f;
    float right_y = init_pos_y - 7.5f;

    float cx = (front_x + back_x) / 2.0f;
    float cy = (left_y + right_y) / 2.0f;
    float len_x = std::abs(front_x - back_x);
    float len_y = std::abs(left_y - right_y);

    //[修改] 把墙削薄，防止向内过度挤压起飞空间
    float wall_thickness = 0.15f;

    Obstacle front_obs;
    front_obs.type = WALL;
    front_obs.position = Eigen::Vector2f(front_x, cy);
    front_obs.angle = M_PI / 2.0f;
    front_obs.length = len_y;
    front_obs.radius = wall_thickness;
    Obstacle back_obs;
    back_obs.type = WALL;
    back_obs.position = Eigen::Vector2f(back_x, cy);
    back_obs.angle = M_PI / 2.0f;
    back_obs.length = len_y;
    back_obs.radius = wall_thickness;
    Obstacle left_obs;
    left_obs.type = WALL;
    left_obs.position = Eigen::Vector2f(cx, left_y);
    left_obs.angle = 0.0f;
    left_obs.length = len_x;
    left_obs.radius = wall_thickness;
    Obstacle right_obs;
    right_obs.type = WALL;
    right_obs.position = Eigen::Vector2f(cx, right_y);
    right_obs.angle = 0.0f;
    right_obs.length = len_x;
    right_obs.radius = wall_thickness;

    static_walls.push_back(front_obs);
    static_walls.push_back(back_obs);
    static_walls.push_back(left_obs);
    static_walls.push_back(right_obs);

    ROS_INFO("✅ 高精电子围栏已激活！边界: X[%.1f, %.1f], Y[%.1f, %.1f]", back_x, front_x, right_y, left_y);
}
// [修复] 点云回调函数：物理隔绝起飞期间的地面污染
void pointcloud_cb(const sensor_msgs::PointCloud2ConstPtr &msg)
{
    if (!flag_init_pos)
        return;

    // IDLE = 0, TAKEOFF = 1, LEG1_AVOID = 2
    // 如果还没进入避障阶段（如起飞中），直接丢弃雷达数据，绝不让地面进地图！
    if (mission_step < 2)
        return;

    pcl::fromROSMsg(*msg, *current_cloud);
    // ROS_INFO_THROTTLE(1.0, "[感知] 收到 ROI 点云: %zu 个障碍点", current_cloud->points.size());
}

// ------------------ 动态二维地图函数 ------------------
OccupancyGrid2D::OccupancyGrid2D()
{
    grid_w = 0;
    grid_h = 0;
    resolution = 0.1f;
    origin_x = 0;
    origin_y = 0;
}

void OccupancyGrid2D::init(float res, float w_m, float h_m, float ox, float oy)
{
    resolution = res;
    origin_x = ox;
    origin_y = oy;
    grid_w = std::ceil(w_m / res);
    grid_h = std::ceil(h_m / res);
    cells.resize(grid_w, std::vector<int>(grid_h, 0));
    ROS_INFO("地图初始化: %dx%d, Res:%.2f", grid_w, grid_h, resolution);
}

bool OccupancyGrid2D::world_to_grid(float wx, float wy, int &gx, int &gy) const
{
    gx = (int)((wx - origin_x) / resolution);
    gy = (int)((wy - origin_y) / resolution);
    return (gx >= 0 && gx < grid_w && gy >= 0 && gy < grid_h);
}

void OccupancyGrid2D::grid_to_world(int gx, int gy, float &wx, float &wy) const
{
    wx = origin_x + (gx + 0.5f) * resolution;
    wy = origin_y + (gy + 0.5f) * resolution;
}

bool OccupancyGrid2D::is_occupied(int gx, int gy) const
{
    if (gx < 0 || gx >= grid_w || gy < 0 || gy >= grid_h)
        return true;
    return cells[gx][gy] > OBS_THRESHOLD;
}

void OccupancyGrid2D::update_with_memory(const std::vector<Obstacle> &static_walls, float drone_r, float safe_margin)
{
    // [新增声明] 用于起飞后清空地面污染
    void clear();
    if (grid_w == 0 || grid_h == 0)
        return;
    bool is_fast_turning = std::abs(current_yaw_rate) > cfg.rotation_gating_threshold;
    if (is_fast_turning)
        return;

    Eigen::Vector2f drone_p(local_pos.pose.pose.position.x, local_pos.pose.pose.position.y);

    // 1. 栅格地图衰减记忆
    for (int i = 0; i < grid_w; ++i)
    {
        for (int j = 0; j < grid_h; ++j)
        {
            if (cells[i][j] > 0)
            {
                float wx, wy;
                grid_to_world(i, j, wx, wy);
                float dist = std::hypot(wx - drone_p.x(), wy - drone_p.y());
                int decay = (dist < 6.0f) ? 10 : 2;
                cells[i][j] = std::max(0, cells[i][j] - decay);
            }
        }
    }

    // 2. 静态虚拟墙 (依然保留适度膨胀，保证 A* 绝不越过场外边界)
    float static_margin = drone_r;
    for (const auto &obs : static_walls)
    {
        if (obs.type == WALL)
        {
            float hl = obs.length / 2.0f;
            Eigen::Vector2f dir(cos(obs.angle), sin(obs.angle));
            Eigen::Vector2f p1 = obs.position - dir * hl;
            Eigen::Vector2f p2 = obs.position + dir * hl;
            float exp = obs.radius + static_margin;
            float exp_sq = exp * exp;

            float min_x = std::min(p1.x(), p2.x()) - exp;
            float max_x = std::max(p1.x(), p2.x()) + exp;
            float min_y = std::min(p1.y(), p2.y()) - exp;
            float max_y = std::max(p1.y(), p2.y()) + exp;

            int min_gx, min_gy, max_gx, max_gy;
            world_to_grid(min_x, min_y, min_gx, min_gy);
            world_to_grid(max_x, max_y, max_gx, max_gy);
            min_gx = std::max(0, min_gx);
            min_gy = std::max(0, min_gy);
            max_gx = std::min(grid_w - 1, max_gx);
            max_gy = std::min(grid_h - 1, max_gy);

            for (int x = min_gx; x <= max_gx; ++x)
            {
                for (int y = min_gy; y <= max_gy; ++y)
                {
                    float wx, wy;
                    grid_to_world(x, y, wx, wy);
                    if (dist_sq_point_to_segment({wx, wy}, p1, p2) <= exp_sq)
                        cells[x][y] = MAX_HEALTH;
                }
            }
        }
    }

    // 3. ROI 动态点云：【彻底取消膨胀】
    // 既然点云本身已经处理过（或表示物理实体），我们直接 1:1 精确映射到栅格！
    if (current_cloud != nullptr)
    {
        for (const auto &pt : current_cloud->points)
        {
            // 过滤掉因为机身自身反射造成的近距离噪点
            if (std::hypot(pt.x - drone_p.x(), pt.y - drone_p.y()) < 0.2f)
                continue;

            int gx, gy;
            if (world_to_grid(pt.x, pt.y, gx, gy))
            {
                cells[gx][gy] = MAX_HEALTH; // 不再外扩，只涂黑这一个格子
            }
        }
    }
}
//[新增实现] 瞬间清空二维地图记忆
void OccupancyGrid2D::clear()
{
    for (auto &col : cells)
    {
        std::fill(col.begin(), col.end(), 0);
    }
}

bool run_astar(const OccupancyGrid2D &grid, Eigen::Vector2f start, Eigen::Vector2f goal, std::vector<Eigen::Vector2f> &out_path)
{
    out_path.clear();
    int sgx, sgy, ggx, ggy;
    if (!grid.world_to_grid(start.x(), start.y(), sgx, sgy) || !grid.world_to_grid(goal.x(), goal.y(), ggx, ggy))
        return false;

    auto find_free = [&](int &cx, int &cy) -> bool
    {
        if (!grid.is_occupied(cx, cy))
            return true;
        std::queue<std::pair<int, int>> q;
        q.push({cx, cy});
        std::vector<bool> vis(grid.grid_w * grid.grid_h, false);
        vis[cx * grid.grid_h + cy] = true;
        int s = 500;
        while (!q.empty() && s--)
        {
            auto cur = q.front();
            q.pop();
            if (!grid.is_occupied(cur.first, cur.second))
            {
                cx = cur.first;
                cy = cur.second;
                return true;
            }
            int dx[] = {1, -1, 0, 0}, dy[] = {0, 0, 1, -1};
            for (int i = 0; i < 4; ++i)
            {
                int nx = cur.first + dx[i], ny = cur.second + dy[i];
                if (nx >= 0 && nx < grid.grid_w && ny >= 0 && ny < grid.grid_h)
                {
                    int nid = nx * grid.grid_h + ny;
                    if (!vis[nid])
                    {
                        vis[nid] = true;
                        q.push({nx, ny});
                    }
                }
            }
        }
        return false;
    };

    if (grid.is_occupied(sgx, sgy))
        find_free(sgx, sgy);
    if (grid.is_occupied(ggx, ggy))
        find_free(ggx, ggy);

    typedef std::pair<float, int> P;
    std::priority_queue<P, std::vector<P>, std::greater<P>> open;
    int max_nodes = grid.grid_w * grid.grid_h;
    std::vector<float> g_cost(max_nodes, 1e9);
    std::vector<int> parent(max_nodes, -1);

    int start_id = sgx * grid.grid_h + sgy;
    int goal_id = ggx * grid.grid_h + ggy;
    g_cost[start_id] = 0;
    open.push({0, start_id});
    bool found = false;
    int iter = 0;

    while (!open.empty() && iter++ < 20000)
    {
        auto top = open.top();
        open.pop();
        int curr = top.second;
        if (curr == goal_id)
        {
            found = true;
            break;
        }
        if (top.first > g_cost[curr] + 100)
            continue;

        int cx = curr / grid.grid_h, cy = curr % grid.grid_h;
        int dx[] = {1, -1, 0, 0, 1, 1, -1, -1}, dy[] = {0, 0, 1, -1, 1, -1, 1, -1};
        float dists[] = {1, 1, 1, 1, 1.4, 1.4, 1.4, 1.4};

        for (int i = 0; i < 8; ++i)
        {
            int nx = cx + dx[i], ny = cy + dy[i];
            if (nx < 0 || nx >= grid.grid_w || ny < 0 || ny >= grid.grid_h || grid.is_occupied(nx, ny))
                continue;
            int nid = nx * grid.grid_h + ny;
            float ng = g_cost[curr] + dists[i];
            if (ng < g_cost[nid])
            {
                g_cost[nid] = ng;
                parent[nid] = curr;
                open.push({ng + std::hypot(nx - ggx, ny - ggy) * cfg.astar_weight, nid});
            }
        }
    }
    if (!found)
        return false;

    int curr = goal_id;
    while (curr != -1)
    {
        float wx, wy;
        grid.grid_to_world(curr / grid.grid_h, curr % grid.grid_h, wx, wy);
        out_path.push_back({wx, wy});
        curr = parent[curr];
    }
    std::reverse(out_path.begin(), out_path.end());
    if (!out_path.empty())
    {
        std::vector<Eigen::Vector2f> s_path;
        s_path.push_back(out_path[0]);
        for (size_t i = 1; i < out_path.size() - 1; ++i)
            if ((out_path[i] - s_path.back()).norm() > 0.6)
                s_path.push_back(out_path[i]);
        s_path.push_back(out_path.back());
        out_path = s_path;
    }
    return true;
}

std::vector<Eigen::Vector2f> BSplinePlanner::generate_smooth_path(const std::vector<Eigen::Vector2f> &cps, int points_per_seg)
{
    std::vector<Eigen::Vector2f> raw_spline;
    if (cps.size() < 2)
        return cps;
    std::vector<Eigen::Vector2f> pts = cps;
    pts.insert(pts.begin(), cps[0]);
    pts.insert(pts.begin(), cps[0]);
    pts.insert(pts.end(), cps.back());
    pts.insert(pts.end(), cps.back());
    for (size_t i = 0; i < pts.size() - 3; ++i)
    {
        for (int j = 0; j < points_per_seg; ++j)
        {
            float u = (float)j / points_per_seg;
            float b0 = (1 - u) * (1 - u) * (1 - u) / 6, b1 = (3 * u * u * u - 6 * u * u + 4) / 6, b2 = (-3 * u * u * u + 3 * u * u + 3 * u + 1) / 6, b3 = u * u * u / 6;
            raw_spline.push_back(b0 * pts[i] + b1 * pts[i + 1] + b2 * pts[i + 2] + b3 * pts[i + 3]);
        }
    }
    raw_spline.push_back(cps.back());
    std::vector<Eigen::Vector2f> resampled_path;
    resampled_path.push_back(raw_spline.front());
    float step_size = 0.1f, dist_accum = 0.0f;
    for (size_t i = 0; i < raw_spline.size() - 1; ++i)
    {
        Eigen::Vector2f p1 = raw_spline[i], p2 = raw_spline[i + 1];
        float segment_len = (p2 - p1).norm();
        if (segment_len < 1e-4)
            continue;
        Eigen::Vector2f dir = (p2 - p1) / segment_len;
        float dist_left = segment_len;
        while (dist_accum + dist_left >= step_size)
        {
            float travel = step_size - dist_accum;
            Eigen::Vector2f new_pt = p1 + dir * travel;
            resampled_path.push_back(new_pt);
            p1 = new_pt;
            dist_left -= travel;
            dist_accum = 0.0f;
        }
        dist_accum += dist_left;
    }
    resampled_path.push_back(raw_spline.back());
    ROS_INFO("长度: %.2f, 原点数: %lu, 平滑后点数: %lu", raw_spline.size() * 0.1f, cps.size(), resampled_path.size());
    return resampled_path;
}

bool is_path_blocked(const std::vector<Eigen::Vector2f> &path, const OccupancyGrid2D &grid, float check_radius)
{
    if (path.empty())
        return true;
    Eigen::Vector2f drone_p(local_pos.pose.pose.position.x, local_pos.pose.pose.position.y);
    int start_idx = 0;
    float min_d = 1e9;
    for (int i = 0; i < path.size(); ++i)
    {
        float d = (path[i] - drone_p).norm();
        if (d < min_d)
        {
            min_d = d;
            start_idx = i;
        }
    }
    for (int i = start_idx; i < path.size(); ++i)
    {
        if ((path[i] - drone_p).norm() > 6.0)
            break;
        int gx, gy;
        if (grid.world_to_grid(path[i].x(), path[i].y(), gx, gy))
            if (grid.cells[gx][gy] > 900)
                return true;
    }
    return false;
}

Eigen::Vector2f get_lookahead_point(const std::vector<Eigen::Vector2f> &path, Eigen::Vector2f curr_pos, float lookahead_dist)
{
    if (path.empty())
        return curr_pos;
    float min_d = 1e9;
    int idx = 0;
    for (int i = 0; i < path.size(); ++i)
    {
        float d = (path[i] - curr_pos).norm();
        if (d < min_d)
        {
            min_d = d;
            idx = i;
        }
    }
    float dist_acc = 0;
    for (int i = idx; i < path.size() - 1; ++i)
    {
        float seg = (path[i + 1] - path[i]).norm();
        if (dist_acc + seg > lookahead_dist)
        {
            float r = (lookahead_dist - dist_acc) / seg;
            return path[i] + (path[i + 1] - path[i]) * r;
        }
        dist_acc += seg;
    }
    return path.back();
}

bool run_vfh_plus(Eigen::Vector2f target, const std::vector<Obstacle> &static_walls, bool &need_replan)
{
    need_replan = false;
    Eigen::Vector2f curr(local_pos.pose.pose.position.x, local_pos.pose.pose.position.y);
    Eigen::Vector2f dir = target - curr;
    float dist = dir.norm();
    if (dist < 0.1)
        return true;

    const int BINS = 72;
    float hist[BINS] = {0};
    float min_obs_d = 1e9;

    // 墙体的硬隔离阈值：机身即可，不需要过度加 margin，否则起飞直接红圈
    float wall_safe_threshold = cfg.uav_radius + 0.1f;
    // 动态点云的隔离阈值
    float cloud_safe_threshold = cfg.uav_radius + cfg.safe_margin + 0.15f;

    // ========================================================================
    // 1. 处理静态虚拟墙 (Geofencing) 斥力 —— [核心修复：解决出墙回不来 Bug]
    // ========================================================================
    for (const auto &o : static_walls)
    {
        if (o.footprint.empty())
            continue;

        // distToPolygon: 如果在多边形内部返回 0，外部返回距离
        // 注意：因为我们画的是线段宽化后的包围盒，这里我们需要重新定义“越界”
        float phys_d = distToPolygon(curr, o.footprint);

        // [关键机制 1]：如果 phys_d == 0，说明飞机已经“陷入”虚拟墙内部，或者完全跑到墙外面去了！
        // 此时我们绝对不能再给它施加斥力把它往外推，必须直接 continue，让 A* 的引力把它拉回正常区域！
        if (phys_d < 0.01f)
        {
            continue;
        }

        // 不把它计入 min_obs_d，防止触发全局死锁不敢起飞
        if (phys_d > 3.0f)
            continue;

        std::vector<float> angles;
        for (const auto &pt : o.footprint)
        {
            float ang = std::atan2(pt.y() - curr.y(), pt.x() - curr.x()) - current_target_yaw;
            while (ang > M_PI)
                ang -= 2 * M_PI;
            while (ang < -M_PI)
                ang += 2 * M_PI;
            angles.push_back(ang);
        }

        // [关键机制 2]：收缩虚拟墙的视网膜膨胀角
        // 以前是用无人机本体半径去膨胀墙，导致很远就觉得没路了。
        // 现在我们把虚拟墙的视觉压迫感调小 (只用 0.1m 的裕度)，让它可以贴墙飞
        float margin_angle = std::asin(std::min(0.85f, (0.1f) / phys_d));
        std::sort(angles.begin(), angles.end());

        float max_gap = angles[0] + 2 * M_PI - angles.back();
        int gap_idx = angles.size() - 1;
        for (size_t i = 0; i < angles.size() - 1; ++i)
        {
            float gap = angles[i + 1] - angles[i];
            if (gap > max_gap)
            {
                max_gap = gap;
                gap_idx = i;
            }
        }

        float start_ang = (gap_idx != angles.size() - 1) ? angles[gap_idx + 1] - margin_angle : angles[0] - margin_angle;
        float end_ang = (gap_idx != angles.size() - 1) ? angles[gap_idx] + 2 * M_PI + margin_angle : angles.back() + margin_angle;

        //[关键机制 3]：将钢铁禁区改为“海绵墙” (Soft Wall)
        // 阈值从 uav_radius + safe_margin (约0.6m) 剧烈压缩到了 0.2m。
        // 并且即使小于 0.2m，也不再给毁灭性的 1000.0，而是给一个 80.0 的强斥力，允许极端情况下强行挤过去。
        float wall_safe_threshold = 0.2f;
        float raw_cost = (phys_d < wall_safe_threshold) ? 80.0f : (10.0f / phys_d);

        int steps = std::ceil((end_ang - start_ang) / (2 * M_PI / BINS));
        for (int k = 0; k <= steps; ++k)
        {
            float a = start_ang + k * (2 * M_PI / BINS);
            int idx = (int)((a + M_PI) / (2 * M_PI) * BINS) % BINS;
            if (idx < 0)
                idx += BINS;
            hist[idx] = std::max(hist[idx], raw_cost);
        }
    }

    // 2. 处理实时动态点云
    if (current_cloud != nullptr)
    {
        for (const auto &pt : current_cloud->points)
        {
            float dx = pt.x - curr.x();
            float dy = pt.y - curr.y();
            float phys_d = std::hypot(dx, dy);

            // [核心修复 2]：扩大自身屏蔽罩到 0.4m！
            // 彻底过滤掉无人机脚底下的 H 标起飞坪和机身噪点！
            if (phys_d < 0.2f || phys_d > 2.0f)
                continue;

            // 只有前方真正挡路的点云，才允许触发紧急制动
            if (phys_d < min_obs_d)
                min_obs_d = phys_d;

            float ang = std::atan2(dy, dx) - current_target_yaw;
            while (ang > M_PI)
                ang -= 2 * M_PI;
            while (ang < -M_PI)
                ang += 2 * M_PI;

            float margin_angle = std::asin(std::min(0.85f, (cfg.uav_radius + cfg.safe_margin) / phys_d));
            int hw = std::ceil(margin_angle / (2 * M_PI / BINS));
            int c_idx = (int)((ang + M_PI) / (2 * M_PI) * BINS) % BINS;
            if (c_idx < 0)
                c_idx += BINS;

            float raw_cost = (phys_d < cloud_safe_threshold) ? 1000.0f : (10.0f / phys_d);
            for (int k = -hw; k <= hw; ++k)
            {
                int idx = (c_idx + k) % BINS;
                if (idx < 0)
                    idx += BINS;
                hist[idx] = std::max(hist[idx], raw_cost);
            }
        }
    }

    // [核心修复 3]：紧急制动现在只对“除了墙以外”的障碍物有效
    if (min_obs_d < cfg.min_safe_dist)
    {
        ROS_WARN_THROTTLE(1.0, "[VFH 紧急制动] 前方/侧方动态障碍物仅 %.2fm", min_obs_d);
        need_replan = true;
        return false;
    }

    float t_yaw = std::atan2(dir.y(), dir.x());
    float rel_t_yaw = t_yaw - current_target_yaw;
    while (rel_t_yaw > M_PI)
        rel_t_yaw -= 2 * M_PI;
    while (rel_t_yaw < -M_PI)
        rel_t_yaw += 2 * M_PI;

    int best_idx = -1;
    float min_c = 1e9;

    // ========================================================
    // [核心修复] VFH 行为权重天平 (可后期移入 YAML)
    // ========================================================
    float weight_target = 0.9f; // 目标牵引权重 (降低！允许偏离 A* 路径)
    float weight_obs = 0.9f;    // 避障斥力权重 (大幅增强！遇到障碍提前绕大弯)
    float weight_smooth = 0.3f; // 运动惯性权重 (防止在两个缝隙间左右横跳)

    // 遍历代价直方图，寻找最优平移方向
    for (int i = 0; i < BINS; ++i)
    {
        if (hist[i] > 100.0f)
            continue; // 绝对禁区拦截 (太近了，此路不通)

        float b_yaw = -M_PI + i * (2 * M_PI / BINS) + (M_PI / BINS) * 0.5f;

        // 1. 计算偏离目标点的代价
        float diff_target = b_yaw - rel_t_yaw;
        while (diff_target > M_PI)
            diff_target -= 2 * M_PI;
        while (diff_target < -M_PI)
            diff_target += 2 * M_PI;

        // 2. 计算偏离上一帧运动方向的代价 (保持走线丝滑)
        float abs_travel_yaw = b_yaw + current_target_yaw; // 当前 bin 的世界绝对朝向
        float diff_last = abs_travel_yaw - last_vfh_yaw;
        while (diff_last > M_PI)
            diff_last -= 2 * M_PI;
        while (diff_last < -M_PI)
            diff_last += 2 * M_PI;

        // 3. 终极代价函数：平衡寻路、避障与平滑
        float c = std::abs(diff_target) * weight_target +
                  hist[i] * weight_obs +
                  std::abs(diff_last) * weight_smooth;

        if (c < min_c)
        {
            min_c = c;
            best_idx = i;
        }
    }
    ROS_INFO_THROTTLE(1.0, "[VFH] 最佳航向 idx: %d, 代价: %.2f, 障碍物代价: %.2f", best_idx, min_c, hist[best_idx]);
    if (best_idx == -1)
    {
        ROS_WARN_THROTTLE(1.0, "[VFH 死锁] 视场内无路可走，请求 A* 重规划");
        need_replan = true;
        return false;
    }

    float final_travel_yaw = -M_PI + best_idx * (2 * M_PI / BINS) + (M_PI / BINS) * 0.5f + current_target_yaw;
    pub_viz_vfh_vectors(t_yaw, final_travel_yaw, curr, hist);
    float speed = cfg.max_speed;
    // 2. [绝对限速]：绝不允许超过设定的最高速度
    if(dist < 0.3f)
        speed = std::min(speed, 0.8f); // 近距离时，最高速度降到 0.8m/s，增加控制精度
    

    // 3. [基于视角的弯道限速]
    float diff = final_travel_yaw - t_yaw;
    while (diff > M_PI)
        diff -= 2 * M_PI;
    while (diff < -M_PI)
        diff += 2 * M_PI;

    if (std::abs(diff) > 1.0f)
        speed *= 0.6f; // 剧烈侧飞躲避时，降速到 60%
    else if (std::abs(diff) > 0.4f)
        speed *= 0.85f; // 轻微绕行时，降速到 85%


    ROS_INFO_THROTTLE(1.0, "[VFH] 目标航向: %.2f°, 当前航向: %.2f°, 航向差: %.2f°, 线速度: %.2fm/s", t_yaw * 180 / M_PI, current_target_yaw * 180 / M_PI, diff * 180 / M_PI, speed);
    // =========================================================================
    // [核心修复]：废除 * 0.05 的极近牵引，改为 1.0 秒的前视远点牵引 (Carrot-on-a-stick)
    // =========================================================================
    float lookahead_time = 1.0f; // 在期望方向上，投影出 1.0 秒后的位置作为飞控目标

    setpoint_raw.position.x = curr.x() + std::cos(final_travel_yaw) * speed * lookahead_time;
    setpoint_raw.position.y = curr.y() + std::sin(final_travel_yaw) * speed * lookahead_time;
    setpoint_raw.yaw = current_target_yaw; // 机头死死锁住

    return false;
}
void pub_viz_vfh_vectors(float t_yaw, float s_yaw, const Eigen::Vector2f &pos, float hist[72])
{
    visualization_msgs::Marker m;
    m.header.frame_id = "map";
    m.ns = "vfh_arrows";
    m.id = 0;
    m.type = visualization_msgs::Marker::ARROW;
    m.action = visualization_msgs::Marker::ADD;
    m.pose.position.x = pos.x();
    m.pose.position.y = pos.y();
    m.pose.position.z = init_pos_z + cfg.takeoff_height;
    m.scale.x = 1.0;
    m.scale.y = 0.05;
    m.scale.z = 0.05;
    m.pose.orientation.w = 1;
    m.color.r = 1.0;
    m.color.a = 1.0;
    tf::quaternionTFToMsg(tf::createQuaternionFromYaw(t_yaw), m.pose.orientation);
    pub_viz_vfh.publish(m);
    m.id = 1;
    m.color.r = 0.0;
    m.color.g = 1.0;
    m.color.b = 0.0;
    tf::quaternionTFToMsg(tf::createQuaternionFromYaw(s_yaw), m.pose.orientation);
    pub_viz_vfh.publish(m);

    visualization_msgs::Marker hist_msg;
    hist_msg.header.frame_id = "map";
    hist_msg.header.stamp = ros::Time::now();
    hist_msg.ns = "vfh_histogram";
    hist_msg.id = 2;
    hist_msg.type = visualization_msgs::Marker::LINE_LIST;
    hist_msg.action = visualization_msgs::Marker::ADD;
    hist_msg.scale.x = 0.02;
    hist_msg.pose.orientation.w = 1.0;

    for (int i = 0; i < 72; ++i)
    {
        float b_yaw = -M_PI + i * (2 * M_PI / 72) + (M_PI / 72) * 0.5f;
        float abs_yaw = b_yaw + current_target_yaw;
        geometry_msgs::Point p1, p2;
        p1.x = pos.x();
        p1.y = pos.y();
        p1.z = init_pos_z + cfg.takeoff_height;
        float cost_len = std::min(std::max(hist[i] * 0.1f, 0.2f), 1.5f);
        p2.x = pos.x() + std::cos(abs_yaw) * cost_len;
        p2.y = pos.y() + std::sin(abs_yaw) * cost_len;
        p2.z = p1.z;
        hist_msg.points.push_back(p1);
        hist_msg.points.push_back(p2);
        std_msgs::ColorRGBA color;
        color.a = 0.8;
        if (hist[i] > 15.0f)
        {
            color.r = 1.0;
            color.g = 0.0;
            color.b = 0.0;
        }
        else
        {
            color.r = 0.0;
            color.g = 1.0;
            color.b = 1.0;
        }
        hist_msg.colors.push_back(color);
        hist_msg.colors.push_back(color);
    }
    pub_viz_vfh.publish(hist_msg);
}

void pub_viz_astar_path(const std::vector<Eigen::Vector2f> &path)
{
    nav_msgs::Path msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "map";
    for (const auto &pt : path)
    {
        geometry_msgs::PoseStamped p;
        p.pose.position.x = pt.x();
        p.pose.position.y = pt.y();
        p.pose.position.z = init_pos_z + cfg.takeoff_height;
        p.pose.orientation.w = 1;
        msg.poses.push_back(p);
    }
    pub_viz_path_raw.publish(msg);
}

void pub_viz_smooth_path(const std::vector<Eigen::Vector2f> &path)
{
    visualization_msgs::MarkerArray ma;
    visualization_msgs::Marker d;
    d.action = 3;
    d.header.frame_id = "map";
    ma.markers.push_back(d);
    for (size_t i = 0; i < path.size(); i += 2)
    {
        visualization_msgs::Marker k;
        k.header.frame_id = "map";
        k.ns = "s";
        k.id = i;
        k.type = 2;
        k.action = 0;
        k.pose.position.x = path[i].x();
        k.pose.position.y = path[i].y();
        k.pose.position.z = init_pos_z + cfg.takeoff_height;
        k.scale.x = 0.15;
        k.scale.y = 0.15;
        k.scale.z = 0.15;
        k.color.b = 1;
        k.color.a = 0.6;
        k.pose.orientation.w = 1;
        ma.markers.push_back(k);
    }
    pub_viz_path_smooth.publish(ma);
}

void pub_viz_grid_map(const OccupancyGrid2D &grid)
{
    nav_msgs::OccupancyGrid msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "map";
    msg.info.resolution = grid.resolution;
    msg.info.width = grid.grid_w;
    msg.info.height = grid.grid_h;
    msg.info.origin.position.x = grid.origin_x;
    msg.info.origin.position.y = grid.origin_y;
    msg.info.origin.orientation.w = 1;
    int total_cells = grid.grid_w * grid.grid_h;
    msg.data.resize(total_cells);
    for (int i = 0; i < total_cells; ++i)
    {
        int x = i % grid.grid_w;
        int y = i / grid.grid_w;
        msg.data[i] = (int8_t)((grid.cells[x][y] > 100) ? 100 : grid.cells[x][y]);
    }
    pub_viz_map.publish(msg);
}

bool execute_avoidance_step(Eigen::Vector2f goal, const std::vector<Obstacle> &static_walls)
{
    Eigen::Vector2f curr(local_pos.pose.pose.position.x, local_pos.pose.pose.position.y);
    float dist_to_goal = (curr - goal).norm();
    ROS_INFO_THROTTLE(1.5, "[导航] 目标: (%.1f, %.1f) | 剩余: %.2fm", goal.x(), goal.y(), dist_to_goal);

    bool blocked = is_path_blocked(global_path_smooth, global_grid, cfg.check_radius_buffer);
    ROS_INFO_THROTTLE(1.0, "[导航] 路径阻塞: %s", blocked ? "是" : "否");
    bool cooldown = (ros::Time::now() - last_replan_time).toSec() > cfg.replan_cooldown;

    if (!has_global_plan || (blocked && cooldown))
    {
        if (run_astar(global_grid, curr, goal, global_path_raw))
        {
            global_path_smooth = BSplinePlanner::generate_smooth_path(global_path_raw, 10);
            

            has_global_plan = true;
            last_replan_time = ros::Time::now();
            vfh_first_run = true;
        }
        else
        {
            ROS_WARN_THROTTLE(1.0, "[A* 死锁] 无法规划路径，原地悬停");
            setpoint_raw.position.x = curr.x();
            setpoint_raw.position.y = curr.y();
            has_global_plan = false;
            return false;
        }
    }

    if (has_global_plan)
    {
        Eigen::Vector2f la = get_lookahead_point(global_path_smooth, curr, cfg.lookahead_dist);
        ROS_INFO_THROTTLE(1.0, "[VFH] 查找前视点: (%.2f, %.2f)", la.x(), la.y());
        bool stuck = false;
        run_vfh_plus(la, static_walls, stuck);
        if (stuck)
            has_global_plan = false;
        if (dist_to_goal < 0.3)
            return true;
    }
    return false;
}

// ============================================================================
// 修复后视觉之字形搜索逻辑 (修复全局标志位Bug)
// ============================================================================
void reset_region_scan()
{
    region_scan_reset_requested = true;
}

void region_scan_velocity(float current_x, float current_y, float x_min, float x_max, float y_min, float y_max, double &vx, double &vy)
{
    static float target_y = 0.0f;
    static float target_x = 0.0f;
    static bool direction_right = true;
    static bool initialized = false;

    if (region_scan_reset_requested)
    {
        initialized = false;
        region_scan_reset_requested = false;
    }

    if (!initialized || target_y < y_min || target_y > y_max)
    {
        target_y = y_min;
        target_x = x_max;
        direction_right = true;
        initialized = true;
    }

    if (std::abs(target_x - current_x) < 0.3f)
    {
        if (direction_right)
        {
            target_y += 0.5f;
            if (target_y > y_max)
                target_y = y_min;
            target_x = x_min;
            direction_right = false;
        }
        else
        {
            target_y += 0.5f;
            if (target_y > y_max)
                target_y = y_min;
            target_x = x_max;
            direction_right = true;
        }
    }

    vx = satfunc((target_x - current_x) * cfg.p_xy, cfg.vel_track_max);
    vy = satfunc((target_y - current_y) * cfg.p_xy, cfg.vel_track_max);
}

enum MissionState
{
    IDLE,
    TAKEOFF,
    LEG1_AVOID,
    TURN1,
    LEG2_CROSS,
    RECOVER,
    TURN2,
    LEG3_AVOID,
    TURN3,
    LEG4_FINAL,
    LANDING_SEARCH,
    LANDING_FOLLOW,
    LANDING_DESCEND,
    FINISHED
};

void state_cb(const mavros_msgs::State::ConstPtr &msg) { mavros_connection_state = *msg; }
void local_pos_cb(const nav_msgs::Odometry::ConstPtr &msg)
{
    local_pos = *msg;
    tf::quaternionMsgToTF(local_pos.pose.pose.orientation, quat);
    double r, p;
    tf::Matrix3x3(quat).getRPY(r, p, current_yaw);
    current_yaw_rate = msg->twist.twist.angular.z;

    if (!flag_init_pos && local_pos.pose.pose.position.z > -0.5)
    {
        init_pos_x = local_pos.pose.pose.position.x;
        init_pos_y = local_pos.pose.pose.position.y;
        init_pos_z = local_pos.pose.pose.position.z;
        init_yaw_take_off = current_yaw;
        flag_init_pos = true;
    }
    flag_init_position = flag_init_pos;
}

int main(int argc, char **argv)
{
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "astar_node");
    ros::NodeHandle nh("~");
    ros::NodeHandle public_nh;

    load_parameters(nh);

    // 初始化动态二维地图
    global_grid.init(cfg.map_resolution, cfg.map_width_m, cfg.map_height_m, cfg.map_origin_x, cfg.map_origin_y);
    ros::Duration(0.5).sleep();

    ros::Subscriber s1 = public_nh.subscribe("mavros/state", 10, state_cb);
    ros::Subscriber s2 = public_nh.subscribe("/mavros/local_position/odom", 10, local_pos_cb);
    ros::Subscriber s3 = public_nh.subscribe("/projected_accumulated_cloud", 1, pointcloud_cb);
    ros::Subscriber s4 = public_nh.subscribe("/ring_center", 10, &RingCrossing::vision_cb, &ring_ctrl);

    pub_setpoint = public_nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 10);
    pub_viz_path_raw = public_nh.advertise<nav_msgs::Path>("/viz/raw_path", 1);
    pub_viz_path_smooth = public_nh.advertise<visualization_msgs::MarkerArray>("/viz/smooth_path", 1);
    pub_viz_vfh = public_nh.advertise<visualization_msgs::Marker>("/viz/vfh_vec", 1);
    pub_viz_map = public_nh.advertise<nav_msgs::OccupancyGrid>("/viz/grid_map", 1, true);

    ros::ServiceClient client_arm = public_nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    ros::ServiceClient client_mode = public_nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

    ros::Subscriber s_yolo = public_nh.subscribe("/yolo/detection", 10, yolo_result_cb);
    ros::Subscriber s_takeoff = public_nh.subscribe("/color_detect/takeoff_color", 10, takeoff_cb);
    ros::Subscriber s_land_clr = public_nh.subscribe("/color_detect/land_color", 10, land_color_cb);
    ros::Subscriber s_land_det = public_nh.subscribe("/color_detect/land_detected", 10, land_detected_cb);
    mission_num_pub = public_nh.advertise<std_msgs::Int8>("/color_detect/mission_num", 10);

    ros::Rate rate(20.0);
    while (ros::ok() && (!mavros_connection_state.connected || local_pos.header.seq == 0))
    {
        ros::spinOnce();
        rate.sleep();
    }

    setpoint_raw.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    setpoint_raw.type_mask = 0b101111111000;
    setpoint_raw.position.x = 0;
    setpoint_raw.position.y = 0;
    setpoint_raw.position.z = 0;
    for (int i = 0; i < 50; ++i)
    {
        pub_setpoint.publish(setpoint_raw);
        ros::spinOnce();
        rate.sleep();
    }

    int input;
    if (if_debug > 0.5)
    {
        std::cout << "输入 1 开始: ";
        std::cin >> input;
    }
    else
        input = 1;
    if (input != 1)
        return 0;

    MissionState state = IDLE;
    ros::Time last_req = ros::Time::now();

    while (ros::ok())
    {
        if (flag_init_pos && static_walls.empty())
            build_static_walls();

        global_grid.update_with_memory(static_walls, cfg.uav_radius, cfg.safe_margin);

        static int map_pub_cnt = 0;
        if (map_pub_cnt++ % 5 == 0)
        {
            pub_viz_grid_map(global_grid);
            if (has_global_plan)
            {
                pub_viz_astar_path(global_path_raw);
                pub_viz_smooth_path(global_path_smooth);
            }
        }

        pub_setpoint.publish(setpoint_raw);
        mission_step = (int)state;
        float dt = 0.05f;

        switch (state)
        {
        case IDLE:
            if (mavros_connection_state.mode != "OFFBOARD" && (ros::Time::now() - last_req > ros::Duration(5.0)))
            {
                mavros_msgs::SetMode srv;
                srv.request.custom_mode = "OFFBOARD";
                client_mode.call(srv);
                last_req = ros::Time::now();
            }
            else if (!mavros_connection_state.armed && (ros::Time::now() - last_req > ros::Duration(5.0)))
            {
                mavros_msgs::CommandBool srv;
                srv.request.value = true;
                client_arm.call(srv);
                last_req = ros::Time::now();
            }
            if (mavros_connection_state.armed)
            {
                if (!flag_init_pos)
                {
                    init_pos_x = local_pos.pose.pose.position.x;
                    init_pos_y = local_pos.pose.pose.position.y;
                    init_pos_z = local_pos.pose.pose.position.z;
                    init_yaw_take_off = current_yaw;
                    current_target_yaw = init_yaw_take_off;
                    flag_init_pos = true;
                }
                state = TAKEOFF;
                ROS_INFO(">>> 起飞");
            }
            break;

        case TAKEOFF:
            mission_num_msg.data = 1;
            mission_num_pub.publish(mission_num_msg);
            setpoint_raw.position.z = init_pos_z + cfg.takeoff_height;
            setpoint_raw.position.x = init_pos_x;
            setpoint_raw.position.y = init_pos_y;
            setpoint_raw.yaw = current_target_yaw;

            // 当到达起飞高度时
            if (std::abs(local_pos.pose.pose.position.z - setpoint_raw.position.z) < 0.2)
            {
                state = LEG1_AVOID;
                has_global_plan = false;

                // [核心修复] 起飞完毕的瞬间，将起飞时的地面噪点彻底擦除，干干净净地出发！
                nh.setParam("/pcl_enable", true);
                global_grid.clear();
                current_cloud->clear();
                
                ROS_INFO(">>> 起飞完成，地图已清洗，平移避障往 WP1");
            }
            break;

        case LEG1_AVOID:
            if (execute_avoidance_step({init_pos_x + cfg.wp1[0], init_pos_y + cfg.wp1[1]}, static_walls))
            {
                state = TURN1;
                ROS_INFO(">>> 到达WP1, 开始转向穿门");
            }
            break;

        case TURN1:
        {
            current_target_yaw = init_yaw_take_off - M_PI/2;
            setpoint_raw.yaw = calc_smooth_yaw(current_target_yaw, setpoint_raw.yaw, dt);
            setpoint_raw.type_mask = 0b101111111000;
            if (get_yaw_diff(current_target_yaw) < 0.1)
            {
                ros::Duration(1.0).sleep();
                state = LEG2_CROSS;
                ring_ctrl.reset();
                ROS_INFO(">>> 视觉穿门");
            }
            break;
        }

        case LEG2_CROSS:
        {
            float d2 = get_dist(cfg.wp2[0], cfg.wp2[1]);
            if (ring_ctrl.compute_cmd(local_pos, current_yaw, setpoint_raw) || d2 < 0.5)
            {
                state = RECOVER;
                ROS_INFO(">>> 恢复避障");
                setpoint_raw.type_mask = 0b101111111000;
                setpoint_raw.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
                setpoint_raw.position.z = init_pos_z + cfg.takeoff_height;
                setpoint_raw.position.x = init_pos_x + cfg.wp2[0];
                setpoint_raw.position.y = init_pos_y + cfg.wp2[1];
            }
            break;
        }

        case RECOVER:
            setpoint_raw.position.x = init_pos_x + cfg.wp2[0];
            setpoint_raw.position.y = init_pos_y + cfg.wp2[1];
            if (get_dist(cfg.wp2[0], cfg.wp2[1]) < 0.3)
            {
                state = TURN2;
                ROS_INFO(">>> 到达WP2, 开始转向");
            }
            break;

        case TURN2:
        {
            current_target_yaw = init_yaw_take_off - M_PI;
            setpoint_raw.yaw = calc_smooth_yaw(current_target_yaw, setpoint_raw.yaw, dt);
            if (get_yaw_diff(current_target_yaw) < 0.1)
            {
                state = LEG3_AVOID;
                has_global_plan = false;
                ROS_INFO(">>> 避障往 WP3");
            }
            break;
        }

        case LEG3_AVOID:
            if (execute_avoidance_step({init_pos_x + cfg.wp3[0], init_pos_y + cfg.wp3[1]}, static_walls))
            {
                state = TURN3;
                ROS_INFO(">>> 到达WP3, 转向准备冲刺终点");
            }
            break;

        case TURN3:
        {
            current_target_yaw = init_yaw_take_off - M_PI / 2.0;
            setpoint_raw.yaw = calc_smooth_yaw(current_target_yaw, setpoint_raw.yaw, dt);
            if (get_yaw_diff(current_target_yaw) < 0.1)
            {
                state = LEG4_FINAL;
                has_global_plan = false;
                ROS_INFO(">>> 平移往 终点区");
            }
            break;
        }

        case LEG4_FINAL:
            if (execute_avoidance_step({init_pos_x + cfg.wp4[0], init_pos_y + cfg.wp4[1]}, static_walls))
            {
                state = LANDING_SEARCH;
                ROS_INFO(">>> 开始蛇形扫描降落区");
                last_req = ros::Time::now();
            }
            break;

        case LANDING_SEARCH:
            mission_num_msg.data = 2;
            mission_num_pub.publish(mission_num_msg);
            {
                float scan_y = init_pos_y + cfg.wp4[1];
                float scan_x_min = init_pos_x + 0.0f;
                float scan_x_max = init_pos_x + 3.6f;
                float scan_y_min = scan_y - 0.5f;
                float scan_y_max = scan_y + 0.5f;

                static MissionState last_state = IDLE;
                if (last_state != LANDING_SEARCH)
                {
                    reset_region_scan();
                    last_state = LANDING_SEARCH;
                }

                setpoint_raw.type_mask = 0b101111100011;
                setpoint_raw.position.z = init_pos_z + cfg.takeoff_height;
                setpoint_raw.yaw = current_target_yaw;

                region_scan_velocity(local_pos.pose.pose.position.x, local_pos.pose.pose.position.y,
                                     scan_x_min, scan_x_max, scan_y_min, scan_y_max,
                                     setpoint_raw.velocity.x, setpoint_raw.velocity.y);
            }
            if (land_detected && takeoff_color == land_color && yolo_result.point.z > 0.5)
            {
                state = LANDING_FOLLOW;
                ROS_INFO(">>> 锁定目标，开始视觉伺服");
                last_req = ros::Time::now();
            }
            break;

        case LANDING_FOLLOW:
        {
            if (land_done)
            {
                setpoint_raw.position.z = local_pos.pose.pose.position.z - 0.15;
                if (local_pos.pose.pose.position.z < init_pos_z + 0.15)
                {
                    state = FINISHED;
                    ROS_INFO(">>> 任务完成, 停桨");
                }
            }
            mission_num_msg.data = 2;
            mission_num_pub.publish(mission_num_msg);

            if (ros::Time::now() - last_req > ros::Duration(cfg.time_threshold) && !land_detected)
            {
                state = LANDING_SEARCH;
                ROS_WARN("目标丢失，重新扫描！");
                break;
            }
            if (land_detected)
            {
                last_req = ros::Time::now();
                float vx = satfunc(yolo_result.point.y * cfg.yolo_follow_kp, cfg.vel_track_max);
                float vy = satfunc(yolo_result.point.x * cfg.yolo_follow_kp, cfg.vel_track_max);
                setpoint_raw.type_mask = 0b100111000011;
                setpoint_raw.velocity.x = vx;
                setpoint_raw.velocity.y = vy;

                if (!land_done)
                    setpoint_raw.position.z = init_pos_z + cfg.takeoff_height;
                if (std::hypot(yolo_result.point.x, yolo_result.point.y) < 0.1)
                {
                    land_done = true;
                    ROS_INFO_ONCE(">>> 已精准对齐目标，垂直下降中...");
                }
            }
            else
            {
                setpoint_raw.type_mask = 0b100111000011;
                setpoint_raw.velocity.x = 0;
                setpoint_raw.velocity.y = 0;
            }
            break;
        }

        case FINISHED:
            setpoint_raw.type_mask = 0;
            return 0;
        }

        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}