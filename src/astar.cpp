/**
 * @file astar.cpp
 * @brief 终极优化版：平滑转向、参数外置、抗震荡、支持方柱矩形精确膨胀避障
 */
#include "astar.h"

#include "ring_crossing.h"

#include <tf/transform_listener.h>

#include <algorithm>
#include <clocale>
#include <cmath>
#include <iostream>

// ============================================================================
// 1. 全局变量
// ============================================================================
float target_x                 = 0.0f;
float target_y                 = 0.0f;
float if_debug                 = 1.0f;

float init_position_x_take_off = 0;
float init_position_y_take_off = 0;
float init_position_z_take_off = 0;
float init_yaw_take_off        = 0;
bool flag_init_position        = false;

ros::Time precision_land_last_time;
bool land_done = false;

std::vector<Obstacle> obstacles;
std::vector<Obstacle> static_walls;
RingCrossing ring_ctrl;

int mission_step = 0;
mavros_msgs::PositionTarget setpoint_raw;
mavros_msgs::State mavros_connection_state;
nav_msgs::Odometry local_pos;
double current_yaw      = 0.0;
double current_yaw_rate = 0.0;
tf::Quaternion quat;

float init_pos_x = 0, init_pos_y = 0, init_pos_z = 0;
bool flag_init_pos = false;

std::vector<Eigen::Vector2f> global_path_raw;
std::vector<Eigen::Vector2f> global_path_smooth;
ros::Time last_replan_time;
bool has_global_plan = false;

OccupancyGrid2D global_grid;
tf::TransformListener *tf_listener = nullptr;

// Scan_Land
std::string takeoff_color          = "";
std::string land_color             = "";
bool land_detected                 = false;
geometry_msgs::PointStamped yolo_result;
std_msgs::Int8 mission_num_msg;
bool search_mode_dir = false;

// VFH 上一帧角度缓存 (用于滤波)
float last_vfh_yaw   = 0.0;
bool vfh_first_run   = true;

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
} cfg;

ros::Publisher pub_setpoint;
ros::Publisher pub_viz_path_raw;
ros::Publisher pub_viz_path_smooth;
ros::Publisher pub_viz_vfh;
ros::Publisher pub_viz_map;
ros::Publisher mission_num_pub;

// ============================================================================
// 3. 辅助函数实现
// ============================================================================

void yolo_result_cb(const geometry_msgs::PointStamped::ConstPtr &msg) {
    yolo_result = *msg;
}
void takeoff_cb(const std_msgs::String::ConstPtr &msg) {
    takeoff_color = msg->data;
}
void land_color_cb(const std_msgs::String::ConstPtr &msg) {
    land_color = msg->data;
}
void land_detected_cb(const std_msgs::Bool::ConstPtr &msg) {
    land_detected = msg->data;
}

void load_parameters(ros::NodeHandle &nh) {
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

    ROS_INFO("=== 参数加载完成 ===");
}

float satfunc(float data, float Max) {
    if (std::abs(data) > Max) return (data > 0) ? Max : -Max;
    return data;
}

float get_dist(float tx, float ty) {
    float dx = (init_pos_x + tx) - local_pos.pose.pose.position.x;
    float dy = (init_pos_y + ty) - local_pos.pose.pose.position.y;
    return std::hypot(dx, dy);
}

float get_yaw_diff(float target_yaw) {
    float diff = target_yaw - current_yaw;
    while (diff > M_PI) diff -= 2 * M_PI;
    while (diff < -M_PI) diff += 2 * M_PI;
    return std::abs(diff);
}

float calc_smooth_yaw(float target_yaw, float current_yaw, float dt) {
    float diff = target_yaw - current_yaw;
    while (diff > M_PI) diff -= 2 * M_PI;
    while (diff < -M_PI) diff += 2 * M_PI;

    float max_step = cfg.max_yaw_rate * dt;
    if (std::abs(diff) < max_step) {
        return target_yaw;
    }
    else {
        return current_yaw + (diff > 0 ? max_step : -max_step);
    }
}

float dist_sq_point_to_segment(const Eigen::Vector2f &p, const Eigen::Vector2f &s_start,
                               const Eigen::Vector2f &s_end) {
    Eigen::Vector2f v = s_end - s_start;
    Eigen::Vector2f w = p - s_start;
    float c1          = w.dot(v);
    if (c1 <= 0) return w.squaredNorm();
    float c2 = v.dot(v);
    if (c2 <= c1) return (p - s_end).squaredNorm();
    float b            = c1 / c2;
    Eigen::Vector2f pb = s_start + b * v;
    return (p - pb).squaredNorm();
}
// ---------------------------------------------------------
// [新增] 2D 计算几何工具集：处理 OBB 投影与凸包
// ---------------------------------------------------------

// 1. 叉积计算：判断 O->A->B 是左拐(>0)还是右拐(<0)
float cross2d(const Eigen::Vector2f &O, const Eigen::Vector2f &A, const Eigen::Vector2f &B) {
    return (A.x() - O.x()) * (B.y() - O.y()) - (A.y() - O.y()) * (B.x() - O.x());
}

// 2. 凸包算法 (Monotone Chain)：将散点连成严格逆时针的凸多边形
std::vector<Eigen::Vector2f> getConvexHull(std::vector<Eigen::Vector2f> pts) {
    if (pts.size() <= 2) return pts;
    std::sort(pts.begin(), pts.end(), [](const Eigen::Vector2f &a, const Eigen::Vector2f &b) {
        return a.x() < b.x() || (std::abs(a.x() - b.x()) < 1e-5 && a.y() < b.y());
    });
    std::vector<Eigen::Vector2f> hull;
    for (const auto &p : pts) {  // 下半凸包
        while (hull.size() >= 2 && cross2d(hull[hull.size() - 2], hull.back(), p) <= 0)
            hull.pop_back();
        hull.push_back(p);
    }
    size_t lower_size = hull.size();
    for (int i = pts.size() - 2; i >= 0; --i) {  // 上半凸包
        while (hull.size() > lower_size && cross2d(hull[hull.size() - 2], hull.back(), pts[i]) <= 0)
            hull.pop_back();
        hull.push_back(pts[i]);
    }
    if (hull.size() > 1) hull.pop_back();  // 移除重复的起点
    return hull;                           // 返回严格逆时针的多边形轮廓
}

// 3. 计算点到凸多边形的最短距离 (内部为0，外部为到边缘的最短距离)
float distToPolygon(const Eigen::Vector2f &pt, const std::vector<Eigen::Vector2f> &poly) {
    if (poly.empty()) return 1e9;
    bool inside       = true;
    float min_dist_sq = 1e9;
    int n             = poly.size();
    for (int i = 0; i < n; ++i) {
        Eigen::Vector2f p1 = poly[i];
        Eigen::Vector2f p2 = poly[(i + 1) % n];
        if (cross2d(p1, p2, pt) < 0) inside = false;  // 右拐说明点在边界外
        float d_sq = dist_sq_point_to_segment(pt, p1, p2);
        if (d_sq < min_dist_sq) min_dist_sq = d_sq;
    }
    return inside ? 0.0f : std::sqrt(min_dist_sq);
}
void build_static_walls()
{
    if (!static_walls.empty())
        return;

    // 根据实机场地测量的距离，建立高精电子围栏 (以起飞点为原点)
    // 假设机头正前方为 +X，左侧为 +Y (标准 ENU 投影)
    float front_x = init_pos_x + 5.3f;
    float back_x = init_pos_x - 0.5f;

    // 如果实际飞行时，发现飞机以为的墙和真实相反，请互换下面两行的 +1.0 和 -7.5
    float left_y = init_pos_y + 1.0f;
    float right_y = init_pos_y - 7.5f;

    float cx = (front_x + back_x) / 2.0f;
    float cy = (left_y + right_y) / 2.0f;
    float len_x = std::abs(front_x - back_x);
    float len_y = std::abs(left_y - right_y);

    float wall_thickness = 0.2f; // 给虚拟墙加点厚度，防止 VFH 越界

    // 1. 前墙 (垂直于 X 轴)
    Obstacle front_obs;
    front_obs.type = WALL;
    front_obs.position = Eigen::Vector2f(front_x, cy);
    front_obs.angle = M_PI / 2.0f;
    front_obs.length = len_y;
    front_obs.radius = wall_thickness;

    // 2. 后墙 (垂直于 X 轴)
    Obstacle back_obs;
    back_obs.type = WALL;
    back_obs.position = Eigen::Vector2f(back_x, cy);
    back_obs.angle = M_PI / 2.0f;
    back_obs.length = len_y;
    back_obs.radius = wall_thickness;

    // 3. 左墙 (平行于 X 轴)
    Obstacle left_obs;
    left_obs.type = WALL;
    left_obs.position = Eigen::Vector2f(cx, left_y);
    left_obs.angle = 0.0f;
    left_obs.length = len_x;
    left_obs.radius = wall_thickness;

    // 4. 右墙 (平行于 X 轴)
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

    ROS_INFO("✅ 高精电子围栏已激活！边界锁死: X[%.1f, %.1f], Y[%.1f, %.1f]", back_x, front_x, right_y, left_y);
}

// ============================================================================
// 4. 感知模块
// ============================================================================
void detection_cb_wrapper(const pcl_detection::ObjectDetectionResult::ConstPtr &msg) {
    if (!flag_init_pos) return;
    if (!msg->success) return;

    obstacles.clear();
    Eigen::Vector2f drone_p(local_pos.pose.pose.position.x, local_pos.pose.pose.position.y);

    int valid_cnt = 0;
    for (const auto &obj : msg->objects) {
        if (!std::isfinite(obj.position.x) || !std::isfinite(obj.position.y)) continue;

        float wx       = obj.position.x;
        float wy       = obj.position.y;
        float dist_rel = std::hypot(wx - drone_p.x(), wy - drone_p.y());

        // 过滤太远或太近的物体
        if (dist_rel > 10.0f || dist_rel < 0.1f) continue;

        Obstacle obs;
        obs.id       = 0;
        obs.type     = obj.type;
        obs.position = Eigen::Vector2f(wx, wy);

        if (obs.type == WALL)  // 类型 0
        {
            obs.radius = cfg.wall_radius;
            obs.width  = obj.width;
            obs.length = obj.width;  // 墙的长度取 width

            // [核心修复] 根据 PCL 传来的平面方程(法向量)，计算墙体真实的偏转角度
            if (obj.plane_coeffs.size() >= 4) {
                float A   = obj.plane_coeffs[0];
                float B   = obj.plane_coeffs[1];
                // 法向量在 XY 平面的投影是 (A, B)，墙面走向垂直于法向量，即 (-B, A)
                obs.angle = std::atan2(A, -B);
            }
            else {
                obs.angle = 0;  // 降级处理
            }
            valid_cnt++;
            obstacles.push_back(obs);
        }
        else if (obs.type == RING)  // 类型 3：环门，不需要建入障碍物地图
        {
            continue;
        }
        else if (obs.type == PILLAR)  // 类型 4：方柱 (OBB 完美解析)
        {
            if (obj.obb_coeffs.size() >= 15) {
                Eigen::Vector3f center(obj.obb_coeffs[0], obj.obb_coeffs[1], obj.obb_coeffs[2]);
                Eigen::Vector3f a0(obj.obb_coeffs[3], obj.obb_coeffs[4], obj.obb_coeffs[5]);
                Eigen::Vector3f a1(obj.obb_coeffs[6], obj.obb_coeffs[7], obj.obb_coeffs[8]);
                Eigen::Vector3f a2(obj.obb_coeffs[9], obj.obb_coeffs[10], obj.obb_coeffs[11]);
                float l = obj.obb_coeffs[12];
                float w = obj.obb_coeffs[13];
                float h = obj.obb_coeffs[14];

                std::vector<Eigen::Vector2f> pts_2d;
                // 计算 8 个顶点的空间坐标，并直接拍扁投影到 2D 平面
                for (int i : {-1, 1}) {
                    for (int j : {-1, 1}) {
                        for (int k : {-1, 1}) {
                            Eigen::Vector3f pt = center + (i * l / 2.0f) * a0 +
                                                 (j * w / 2.0f) * a1 + (k * h / 2.0f) * a2;
                            pts_2d.push_back(Eigen::Vector2f(pt.x(), pt.y()));
                        }
                    }
                }
                // 计算真正的二维阴影轮廓
                obs.footprint = getConvexHull(pts_2d);
                obs.position  = Eigen::Vector2f(center.x(), center.y());
                valid_cnt++;
                obstacles.push_back(obs);
            }
        }
    }

    // [诊断雷达]：只要节点接通了，终端每秒都会刷出这句话。如果一直不印，说明依然是话题或MD5问题。
    ROS_INFO_THROTTLE(1.0, "[A* 避障节点] 收到 PCL 数据: 包含 %zu 个物体, 成功解析绘制 %d 个",
                      msg->objects.size(), valid_cnt);
}

// ------------------ 地图与规划核心函数 ------------------
OccupancyGrid2D::OccupancyGrid2D() {
    resolution = 0.1f;
    origin_x   = -10.0f;
    origin_y   = -10.0f;
    for (int i = 0; i < GRID_W; ++i)
        for (int j = 0; j < GRID_H; ++j) cells[i][j] = 0;
}
bool OccupancyGrid2D::world_to_grid(float wx, float wy, int &gx, int &gy) const {
    gx = (int)((wx - origin_x) / resolution);
    gy = (int)((wy - origin_y) / resolution);
    return (gx >= 0 && gx < GRID_W && gy >= 0 && gy < GRID_H);
}
void OccupancyGrid2D::grid_to_world(int gx, int gy, float &wx, float &wy) const {
    wx = origin_x + (gx + 0.5f) * resolution;
    wy = origin_y + (gy + 0.5f) * resolution;
}
bool OccupancyGrid2D::is_occupied(int gx, int gy) const {
    if (gx < 0 || gx >= GRID_W || gy < 0 || gy >= GRID_H) return true;
    return cells[gx][gy] > OBS_THRESHOLD;
}

void OccupancyGrid2D::update_with_memory(const std::vector<Obstacle> &obstacles, float drone_r,
                                         float safe_margin) {
    Eigen::Vector2f drone_p(local_pos.pose.pose.position.x, local_pos.pose.pose.position.y);
    bool is_fast_turning = std::abs(current_yaw_rate) > cfg.rotation_gating_threshold;

    // [诊断雷达]：如果你发现地图画不出来，看看是不是这里一直报错！
    if (is_fast_turning) {
        ROS_WARN_THROTTLE(2.0, "[A* 建图警告] 无人机角速度过大 (%.2f)，为防重影已暂停建图！",
                          current_yaw_rate);
    }

    // 地图记忆衰减逻辑 (保持不变)
    for (int i = 0; i < GRID_W; ++i) {
        for (int j = 0; j < GRID_H; ++j) {
            if (cells[i][j] > 0) {
                float wx, wy;
                grid_to_world(i, j, wx, wy);
                float dist = std::hypot(wx - drone_p.x(), wy - drone_p.y());
                int decay  = 0;
                if (dist < 6.0f)
                    decay = is_fast_turning ? 50 : 10;
                else if (dist > 20.0f)
                    decay = 2;
                else
                    decay = is_fast_turning ? 5 : 0;
                cells[i][j] = std::max(0, cells[i][j] - decay);
            }
        }
    }

    // 如果旋转过快，地图只衰减不新增
    if (is_fast_turning) return;

    float total_margin = drone_r + safe_margin;
    for (const auto &obs : obstacles) {
        if ((obs.position - drone_p).norm() < 0.2f) continue;

        if (obs.type == PILLAR) {
            if (obs.footprint.empty()) continue;

            // 1. 框出这个多边形的极大粗略 AABB (加上安全膨胀区) 减少遍历范围
            float min_x = 1e9, max_x = -1e9, min_y = 1e9, max_y = -1e9;
            for (const auto &p : obs.footprint) {
                if (p.x() < min_x) min_x = p.x();
                if (p.x() > max_x) max_x = p.x();
                if (p.y() < min_y) min_y = p.y();
                if (p.y() > max_y) max_y = p.y();
            }
            min_x -= total_margin;
            max_x += total_margin;
            min_y -= total_margin;
            max_y += total_margin;

            int min_gx, min_gy, max_gx, max_gy;
            world_to_grid(min_x, min_y, min_gx, min_gy);
            world_to_grid(max_x, max_y, max_gx, max_gy);
            min_gx = std::max(0, min_gx);
            min_gy = std::max(0, min_gy);
            max_gx = std::min(GRID_W - 1, max_gx);
            max_gy = std::min(GRID_H - 1, max_gy);

            // 2. 只有距离多边形物理边缘 <= total_margin 的栅格，才被精确涂黑
            for (int x = min_gx; x <= max_gx; ++x) {
                for (int y = min_gy; y <= max_gy; ++y) {
                    float wx, wy;
                    grid_to_world(x, y, wx, wy);
                    if (distToPolygon({wx, wy}, obs.footprint) <= total_margin) {
                        cells[x][y] = MAX_HEALTH;
                    }
                }
            }
        }
        else if (obs.type == WALL) {
            float hl = obs.length / 2.0f;
            Eigen::Vector2f dir(cos(obs.angle), sin(obs.angle));
            Eigen::Vector2f p1 = obs.position - dir * hl;
            Eigen::Vector2f p2 = obs.position + dir * hl;
            float exp          = obs.radius + total_margin;
            float exp_sq       = exp * exp;

            float min_x        = std::min(p1.x(), p2.x()) - exp;
            float max_x        = std::max(p1.x(), p2.x()) + exp;
            float min_y        = std::min(p1.y(), p2.y()) - exp;
            float max_y        = std::max(p1.y(), p2.y()) + exp;

            int min_gx, min_gy, max_gx, max_gy;
            world_to_grid(min_x, min_y, min_gx, min_gy);
            world_to_grid(max_x, max_y, max_gx, max_gy);

            min_gx = std::max(0, min_gx);
            min_gy = std::max(0, min_gy);
            max_gx = std::min(GRID_W - 1, max_gx);
            max_gy = std::min(GRID_H - 1, max_gy);
            for (int x = min_gx; x <= max_gx; ++x) {
                for (int y = min_gy; y <= max_gy; ++y) {
                    float wx, wy;
                    grid_to_world(x, y, wx, wy);
                    if (dist_sq_point_to_segment({wx, wy}, p1, p2) <= exp_sq)
                        cells[x][y] = MAX_HEALTH;
                }
            }
        }
    }
}

bool run_astar(const OccupancyGrid2D &grid, Eigen::Vector2f start, Eigen::Vector2f goal,
               std::vector<Eigen::Vector2f> &out_path) {
    out_path.clear();
    int sgx, sgy, ggx, ggy;
    if (!grid.world_to_grid(start.x(), start.y(), sgx, sgy) ||
        !grid.world_to_grid(goal.x(), goal.y(), ggx, ggy))
        return false;
    auto find_free = [&](int &cx, int &cy) -> bool {
        if (!grid.is_occupied(cx, cy)) return true;
        std::queue<std::pair<int, int>> q;
        q.push({cx, cy});
        bool vis[200][200] = {false};
        vis[cx][cy]        = true;
        int s              = 500;
        while (!q.empty() && s--) {
            auto cur = q.front();
            q.pop();
            if (!grid.is_occupied(cur.first, cur.second)) {
                cx = cur.first;
                cy = cur.second;
                return true;
            }
            int dx[] = {1, -1, 0, 0}, dy[] = {0, 0, 1, -1};
            for (int i = 0; i < 4; ++i) {
                int nx = cur.first + dx[i], ny = cur.second + dy[i];
                if (nx >= 0 && nx < 200 && ny >= 0 && ny < 200 && !vis[nx][ny]) {
                    vis[nx][ny] = true;
                    q.push({nx, ny});
                }
            }
        }
        return false;
    };
    if (grid.is_occupied(sgx, sgy)) find_free(sgx, sgy);
    if (grid.is_occupied(ggx, ggy)) find_free(ggx, ggy);
    typedef std::pair<float, int> P;
    std::priority_queue<P, std::vector<P>, std::greater<P>> open;
    static float g_cost[40000];
    static int parent[40000];
    std::fill(g_cost, g_cost + 40000, 1e9);
    std::fill(parent, parent + 40000, -1);
    int start_id = sgx * 200 + sgy, goal_id = ggx * 200 + ggy;
    g_cost[start_id] = 0;
    open.push({0, start_id});
    bool found = false;
    int iter   = 0;
    while (!open.empty() && iter++ < 20000) {
        auto top = open.top();
        open.pop();
        int curr = top.second;
        if (curr == goal_id) {
            found = true;
            break;
        }
        if (top.first > g_cost[curr] + 100) continue;
        int cx = curr / 200, cy = curr % 200;
        int dx[] = {1, -1, 0, 0, 1, 1, -1, -1}, dy[] = {0, 0, 1, -1, 1, -1, 1, -1};
        float dists[] = {1, 1, 1, 1, 1.4, 1.4, 1.4, 1.4};
        for (int i = 0; i < 8; ++i) {
            int nx = cx + dx[i], ny = cy + dy[i];
            if (nx < 0 || nx >= 200 || ny < 0 || ny >= 200 || grid.is_occupied(nx, ny)) continue;
            int nid  = nx * 200 + ny;
            float ng = g_cost[curr] + dists[i];
            if (ng < g_cost[nid]) {
                g_cost[nid] = ng;
                parent[nid] = curr;
                open.push({ng + std::hypot(nx - ggx, ny - ggy) * cfg.astar_weight, nid});
            }
        }
    }
    if (!found) return false;
    int curr = goal_id;
    while (curr != -1) {
        float wx, wy;
        grid.grid_to_world(curr / 200, curr % 200, wx, wy);
        out_path.push_back({wx, wy});
        curr = parent[curr];
    }
    std::reverse(out_path.begin(), out_path.end());
    if (!out_path.empty()) {
        std::vector<Eigen::Vector2f> s_path;
        s_path.push_back(out_path[0]);
        for (size_t i = 1; i < out_path.size() - 1; ++i)
            if ((out_path[i] - s_path.back()).norm() > 0.6) s_path.push_back(out_path[i]);
        s_path.push_back(out_path.back());
        out_path = s_path;
    }
    return true;
}

std::vector<Eigen::Vector2f>
BSplinePlanner::generate_smooth_path(const std::vector<Eigen::Vector2f> &cps, int points_per_seg) {
    std::vector<Eigen::Vector2f> raw_spline;
    if (cps.size() < 2) return cps;

    // 1. 传统 B-Spline 生成 (保证几何上的 C2 连续，即曲率连续)
    std::vector<Eigen::Vector2f> pts = cps;
    pts.insert(pts.begin(), cps[0]);
    pts.insert(pts.begin(), cps[0]);
    pts.insert(pts.end(), cps.back());
    pts.insert(pts.end(), cps.back());

    for (size_t i = 0; i < pts.size() - 3; ++i) {
        for (int j = 0; j < points_per_seg; ++j) {
            float u  = (float)j / points_per_seg;
            float b0 = (1 - u) * (1 - u) * (1 - u) / 6;
            float b1 = (3 * u * u * u - 6 * u * u + 4) / 6;
            float b2 = (-3 * u * u * u + 3 * u * u + 3 * u + 1) / 6;
            float b3 = u * u * u / 6;
            raw_spline.push_back(b0 * pts[i] + b1 * pts[i + 1] + b2 * pts[i + 2] + b3 * pts[i + 3]);
        }
    }
    raw_spline.push_back(cps.back());

    // 2. [核心运动学优化]：等弧长重采样 (Arc-length Resampling)
    // 消除参数曲线点分布不均导致的加速度阶跃，使前视点匀速滑移，大幅提升无人机飞行丝滑度
    std::vector<Eigen::Vector2f> resampled_path;
    resampled_path.push_back(raw_spline.front());

    float step_size  = 0.1f;  // 严格规定每 0.1 米一个航点
    float dist_accum = 0.0f;

    for (size_t i = 0; i < raw_spline.size() - 1; ++i) {
        Eigen::Vector2f p1 = raw_spline[i];
        Eigen::Vector2f p2 = raw_spline[i + 1];
        float segment_len  = (p2 - p1).norm();
        if (segment_len < 1e-4) continue;

        Eigen::Vector2f dir = (p2 - p1) / segment_len;
        float dist_left     = segment_len;

        while (dist_accum + dist_left >= step_size) {
            float travel           = step_size - dist_accum;
            Eigen::Vector2f new_pt = p1 + dir * travel;
            resampled_path.push_back(new_pt);
            p1 = new_pt;
            dist_left -= travel;
            dist_accum = 0.0f;
        }
        dist_accum += dist_left;
    }
    resampled_path.push_back(raw_spline.back());

    return resampled_path;
}

bool is_path_blocked(const std::vector<Eigen::Vector2f> &path, const OccupancyGrid2D &grid,
                     float check_radius) {
    if (path.empty()) return true;
    Eigen::Vector2f drone_p(local_pos.pose.pose.position.x, local_pos.pose.pose.position.y);
    int start_idx = 0;
    float min_d   = 1e9;
    for (int i = 0; i < path.size(); ++i) {
        float d = (path[i] - drone_p).norm();
        if (d < min_d) {
            min_d     = d;
            start_idx = i;
        }
    }
    for (int i = start_idx; i < path.size(); ++i) {
        if ((path[i] - drone_p).norm() > 6.0) break;
        int gx, gy;
        if (grid.world_to_grid(path[i].x(), path[i].y(), gx, gy))
            if (grid.cells[gx][gy] > 900) return true;
    }
    return false;
}

Eigen::Vector2f get_lookahead_point(const std::vector<Eigen::Vector2f> &path,
                                    Eigen::Vector2f curr_pos, float lookahead_dist) {
    if (path.empty()) return curr_pos;
    float min_d = 1e9;
    int idx     = 0;
    for (int i = 0; i < path.size(); ++i) {
        float d = (path[i] - curr_pos).norm();
        if (d < min_d) {
            min_d = d;
            idx   = i;
        }
    }
    float dist_acc = 0;
    for (int i = idx; i < path.size() - 1; ++i) {
        float seg = (path[i + 1] - path[i]).norm();
        if (dist_acc + seg > lookahead_dist) {
            float r = (lookahead_dist - dist_acc) / seg;
            return path[i] + (path[i + 1] - path[i]) * r;
        }
        dist_acc += seg;
    }
    return path.back();
}

bool run_vfh_plus(Eigen::Vector2f target, const std::vector<Obstacle> &obs, bool &need_replan) {
    need_replan = false;
    Eigen::Vector2f curr(local_pos.pose.pose.position.x, local_pos.pose.pose.position.y);
    Eigen::Vector2f dir = target - curr;
    float dist          = dir.norm();
    if (dist < 0.2) return true;

    if (vfh_first_run) {
        last_vfh_yaw  = current_yaw;
        vfh_first_run = false;
    }

    const int BINS   = 72;
    float hist[BINS] = {0};
    float min_obs_d  = 1e9;  // 仅记录无人机到物体的【物理真实距离】

    for (const auto &o : obs) {
        if (o.type == RING) continue;

        if (o.type == PILLAR) {
            if (o.footprint.empty()) continue;

            // 1. 计算无人机到该多边形(物理实体)的最短距离
            float phys_d = distToPolygon(curr, o.footprint);
            if (phys_d < min_obs_d) min_obs_d = phys_d;
            if (phys_d > 3.0 || phys_d < 0.05) continue;

            // 2. 精确计算 FOV 视角阻挡：遍历多边形的每一个角点求角度！
            std::vector<float> angles;
            for (const auto &pt : o.footprint) {
                float ang = std::atan2(pt.y() - curr.y(), pt.x() - curr.x()) - current_yaw;
                while (ang > M_PI) ang -= 2 * M_PI;
                while (ang < -M_PI) ang += 2 * M_PI;
                angles.push_back(ang);
            }

            // 添加无人机尺寸和安全系数导致的视角膨胀补偿
            float margin_angle =
                std::asin(std::min(1.0f, (cfg.uav_radius + cfg.safe_margin) / (phys_d + 0.1f)));

            std::sort(angles.begin(), angles.end());
            float max_gap = angles[0] + 2 * M_PI - angles.back();
            int gap_idx   = angles.size() - 1;
            for (size_t i = 0; i < angles.size() - 1; ++i) {
                float gap = angles[i + 1] - angles[i];
                if (gap > max_gap) {
                    max_gap = gap;
                    gap_idx = i;
                }
            }

            float start_ang, end_ang;
            if (gap_idx != angles.size() - 1) {
                start_ang = angles[gap_idx + 1] - margin_angle;
                end_ang   = angles[gap_idx] + 2 * M_PI + margin_angle;
            }
            else {
                start_ang = angles[0] - margin_angle;
                end_ang   = angles.back() + margin_angle;
            }

            int steps = std::ceil((end_ang - start_ang) / (2 * M_PI / BINS));
            for (int k = 0; k <= steps; ++k) {
                float a = start_ang + k * (2 * M_PI / BINS);
                int idx = (int)((a + M_PI) / (2 * M_PI) * BINS) % BINS;
                if (idx < 0) idx += BINS;
                hist[idx] += 10.0f / (phys_d + 0.1f);
            }
        }
        else if (o.type == WALL) {
            // 同样修复墙体的物理真实距离计算
            float hl = o.length / 2.0f;
            Eigen::Vector2f w_dir(cos(o.angle), sin(o.angle));
            Eigen::Vector2f p1 = o.position - w_dir * hl;
            Eigen::Vector2f p2 = o.position + w_dir * hl;

            // 线段到无人机的几何距离 减去 墙体自身厚度
            float phys_d       = std::sqrt(dist_sq_point_to_segment(curr, p1, p2)) - o.radius;
            if (phys_d < 0) phys_d = 0;

            if (phys_d < min_obs_d) min_obs_d = phys_d;
            if (phys_d > 3.0 || phys_d < 0.05) continue;

            Eigen::Vector2f to_obs = o.position - curr;
            float angle            = std::atan2(to_obs.y(), to_obs.x()) - current_yaw;
            while (angle > M_PI) angle -= 2 * M_PI;
            while (angle < -M_PI) angle += 2 * M_PI;

            float w_ang = std::asin(
                std::min(1.0f, (o.radius + cfg.uav_radius + cfg.safe_margin) / (phys_d + 0.1f)));
            int c_idx = (int)((angle + M_PI) / (2 * M_PI) * BINS) % BINS;
            int hw    = (int)(w_ang / (2 * M_PI) * BINS) + 1;
            for (int k = c_idx - hw; k <= c_idx + hw; ++k)
                hist[(k + BINS) % BINS] += 10.0f / (phys_d + 0.1f);
        }
    }

    // [新增诊断输出] 如果真的触发了紧急制动，大声喊出来！
    if (min_obs_d < cfg.min_safe_dist) {
        ROS_WARN_THROTTLE(1.0, "[VFH] 触发紧急制动！距物理障碍物仅 %.2fm (阈值: %.2f)", min_obs_d,
                          cfg.min_safe_dist);
        need_replan = true;
        return false;
    }

    float t_yaw     = std::atan2(dir.y(), dir.x());
    float rel_t_yaw = t_yaw - current_yaw;
    while (rel_t_yaw > M_PI) rel_t_yaw -= 2 * M_PI;
    while (rel_t_yaw < -M_PI) rel_t_yaw += 2 * M_PI;

    int best_idx = -1;
    float min_c  = 1e9;
    for (int i = 0; i < BINS; ++i) {
        if (hist[i] > 15.0) continue;
        float b_yaw     = -M_PI + i * (2 * M_PI / BINS) + (M_PI / BINS) * 0.5f;
        float diff_last = std::abs(b_yaw - (last_vfh_yaw - current_yaw));
        while (diff_last > M_PI) diff_last -= 2 * M_PI;
        float c = std::abs(b_yaw - rel_t_yaw) + hist[i] * 0.1f + std::abs(diff_last) * 0.5f;
        if (c < min_c) {
            min_c    = c;
            best_idx = i;
        }
    }

    if (best_idx == -1) {
        ROS_WARN_THROTTLE(1.0, "[VFH] 局部死锁！所有方向被封死，请求重新 A*");
        need_replan = true;
        return false;
    }

    float selected_yaw = -M_PI + best_idx * (2 * M_PI / BINS) + (M_PI / BINS) * 0.5f + current_yaw;
    float diff         = selected_yaw - last_vfh_yaw;
    while (diff > M_PI) diff -= 2 * M_PI;
    while (diff < -M_PI) diff += 2 * M_PI;

    float final_yaw = last_vfh_yaw + diff * cfg.yaw_smooth_weight;
    last_vfh_yaw    = final_yaw;

    pub_viz_vfh_vectors(t_yaw, final_yaw, curr);

    // [运动学控制平滑] 根据转弯角度自动调节前馈速度
    float speed = std::min(cfg.max_speed, dist);
    if (std::abs(diff) > 0.8)
        speed *= 0.2;  // 遇急弯深踩刹车
    else if (std::abs(diff) > 0.3)
        speed *= 0.6;  // 缓弯微收油门

    setpoint_raw.position.x = curr.x() + std::cos(final_yaw) * speed * 0.5;
    setpoint_raw.position.y = curr.y() + std::sin(final_yaw) * speed * 0.5;
    setpoint_raw.position.z = init_pos_z + cfg.takeoff_height;
    setpoint_raw.yaw        = final_yaw;
    return false;
}

void pub_viz_astar_path(const std::vector<Eigen::Vector2f> &path) {
    nav_msgs::Path msg;
    msg.header.stamp    = ros::Time::now();
    msg.header.frame_id = "map";
    for (const auto &pt : path) {
        geometry_msgs::PoseStamped p;
        p.pose.position.x    = pt.x();
        p.pose.position.y    = pt.y();
        p.pose.position.z    = init_pos_z + cfg.takeoff_height;
        p.pose.orientation.w = 1;
        msg.poses.push_back(p);
    }
    pub_viz_path_raw.publish(msg);
}
void pub_viz_smooth_path(const std::vector<Eigen::Vector2f> &path) {
    visualization_msgs::MarkerArray ma;
    visualization_msgs::Marker d;
    d.action          = 3;
    d.header.frame_id = "map";
    ma.markers.push_back(d);
    for (size_t i = 0; i < path.size(); i += 2) {
        visualization_msgs::Marker k;
        k.header.frame_id    = "map";
        k.ns                 = "s";
        k.id                 = i;
        k.type               = 2;
        k.action             = 0;
        k.pose.position.x    = path[i].x();
        k.pose.position.y    = path[i].y();
        k.pose.position.z    = init_pos_z + cfg.takeoff_height;
        k.scale.x            = 0.15;
        k.scale.y            = 0.15;
        k.scale.z            = 0.15;
        k.color.b            = 1;
        k.color.a            = 0.6;
        k.pose.orientation.w = 1;
        ma.markers.push_back(k);
    }
    pub_viz_path_smooth.publish(ma);
}
void pub_viz_vfh_vectors(float t_yaw, float s_yaw, const Eigen::Vector2f &pos) {
    visualization_msgs::Marker m;
    m.header.frame_id    = "map";
    m.ns                 = "vfh";
    m.id                 = 0;
    m.type               = 0;
    m.action             = 0;
    m.pose.position.x    = pos.x();
    m.pose.position.y    = pos.y();
    m.pose.position.z    = init_pos_z + cfg.takeoff_height;
    m.scale.x            = 1.0;
    m.scale.y            = 0.05;
    m.scale.z            = 0.05;
    m.pose.orientation.w = 1;
    m.color.r            = 1;
    m.color.a            = 1;
    tf::quaternionTFToMsg(tf::createQuaternionFromYaw(t_yaw), m.pose.orientation);
    pub_viz_vfh.publish(m);
    m.id      = 1;
    m.color.r = 0;
    m.color.g = 1;
    tf::quaternionTFToMsg(tf::createQuaternionFromYaw(s_yaw), m.pose.orientation);
    pub_viz_vfh.publish(m);
}
void pub_viz_grid_map(const OccupancyGrid2D &grid) {
    nav_msgs::OccupancyGrid msg;
    msg.header.stamp              = ros::Time::now();
    msg.header.frame_id           = "map";
    msg.info.resolution           = grid.resolution;
    msg.info.width                = 200;
    msg.info.height               = 200;
    msg.info.origin.position.x    = grid.origin_x;
    msg.info.origin.position.y    = grid.origin_y;
    msg.info.origin.orientation.w = 1;
    msg.data.resize(40000);
    for (int i = 0; i < 40000; ++i)
        msg.data[i] =
            (int8_t)((grid.cells[i % 200][i / 200] > 100) ? 100 : grid.cells[i % 200][i / 200]);
    pub_viz_map.publish(msg);
}

// ============================================================================
// 逻辑封装：执行单步避障 (引入 all_obs)
// ============================================================================
bool execute_avoidance_step(Eigen::Vector2f goal, const std::vector<Obstacle> &all_obs)
{
    Eigen::Vector2f curr(local_pos.pose.pose.position.x, local_pos.pose.pose.position.y);

    // 注意：地图的 update_with_memory 已经挪到主循环中统一执行，彻底解决“双重衰减”Bug

    bool blocked = is_path_blocked(global_path_smooth, global_grid, cfg.check_radius_buffer);
    bool cooldown = (ros::Time::now() - last_replan_time).toSec() > cfg.replan_cooldown;

    if (!has_global_plan || (blocked && cooldown))
    {
        if (blocked && has_global_plan)
            ROS_WARN("路径被动态障碍物截断，重规划...");
        if (run_astar(global_grid, curr, goal, global_path_raw))
        {
            global_path_smooth = BSplinePlanner::generate_smooth_path(global_path_raw, 10);
            has_global_plan = true;
            last_replan_time = ros::Time::now();
            vfh_first_run = true;
            ROS_INFO("规划成功，平滑路径点: %lu", global_path_smooth.size());
        }
        else
        {
            setpoint_raw.position.x = curr.x();
            setpoint_raw.position.y = curr.y();
            has_global_plan = false;
            return false;
        }
    }

    if (has_global_plan)
    {
        Eigen::Vector2f la = get_lookahead_point(global_path_smooth, curr, cfg.lookahead_dist);
        bool stuck = false;

        // VFH 也使用合并后的障碍物，确保不会冲出虚拟墙
        bool reached = run_vfh_plus(la, all_obs, stuck);

        if (stuck)
            has_global_plan = false;
        if ((curr - goal).norm() < 0.3)
            return true;
    }
    return false;
}

enum MissionState {
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

void state_cb(const mavros_msgs::State::ConstPtr &msg) {
    mavros_connection_state = *msg;
}
void local_pos_cb(const nav_msgs::Odometry::ConstPtr &msg) {
    local_pos = *msg;
    tf::quaternionMsgToTF(local_pos.pose.pose.orientation, quat);
    double r, p;
    tf::Matrix3x3(quat).getRPY(r, p, current_yaw);
    current_yaw_rate = msg->twist.twist.angular.z;

    if (!flag_init_pos && local_pos.pose.pose.position.z > -0.5) {
        init_pos_x        = local_pos.pose.pose.position.x;
        init_pos_y        = local_pos.pose.pose.position.y;
        init_pos_z        = local_pos.pose.pose.position.z;
        init_yaw_take_off = current_yaw;
        flag_init_pos     = true;
    }
    flag_init_position = flag_init_pos;
}

int main(int argc, char **argv) {
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "astar_node");
    ros::NodeHandle nh("~");
    ros::NodeHandle public_nh;
    load_parameters(nh);

    // tf_listener 保留定义避免外部依赖崩溃，但核心感知已不再使用
    tf_listener = new tf::TransformListener();
    ros::Duration(0.5).sleep();

    // 必须改回 /pcl_detection/result，因为你的 launch/yaml 里配的就是这个
    ros::Subscriber s1 = public_nh.subscribe("mavros/state", 10, state_cb);
    ros::Subscriber s2 = public_nh.subscribe("/mavros/local_position/odom", 10, local_pos_cb);
    ros::Subscriber s3 = public_nh.subscribe("/pcl_detection/result", 10, detection_cb_wrapper);
    ros::Subscriber s4 =
        public_nh.subscribe("/ring_center", 10, &RingCrossing::vision_cb, &ring_ctrl);

    pub_setpoint =
        public_nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 10);
    pub_viz_path_raw = public_nh.advertise<nav_msgs::Path>("/viz/raw_path", 1);
    pub_viz_path_smooth =
        public_nh.advertise<visualization_msgs::MarkerArray>("/viz/smooth_path", 1);
    pub_viz_vfh = public_nh.advertise<visualization_msgs::Marker>("/viz/vfh_vec", 1);
    pub_viz_map = public_nh.advertise<nav_msgs::OccupancyGrid>("/viz/grid_map", 1, true);

    ros::ServiceClient client_arm =
        public_nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    ros::ServiceClient client_mode =
        public_nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

    ros::Subscriber s_yolo     = public_nh.subscribe("/yolo/detection", 10, yolo_result_cb);
    ros::Subscriber s_takeoff  = public_nh.subscribe("/color_detect/takeoff_color", 10, takeoff_cb);
    ros::Subscriber s_land_clr = public_nh.subscribe("/color_detect/land_color", 10, land_color_cb);
    ros::Subscriber s_land_det =
        public_nh.subscribe("/color_detect/land_detected", 10, land_detected_cb);

    mission_num_pub = public_nh.advertise<std_msgs::Int8>("/color_detect/mission_num", 10);

    ros::Rate rate(20.0);
    while (ros::ok() && (!mavros_connection_state.connected || local_pos.header.seq == 0)) {
        ros::spinOnce();
        rate.sleep();
    }

    setpoint_raw.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    setpoint_raw.type_mask        = 0b101111111000;
    setpoint_raw.position.x       = 0;
    setpoint_raw.position.y       = 0;
    setpoint_raw.position.z       = 0;
    for (int i = 0; i < 50; ++i) {
        pub_setpoint.publish(setpoint_raw);
        ros::spinOnce();
        rate.sleep();
    }

    int input;
    if (if_debug > 0.5) {
        std::cout << "输入 1 开始: ";
        std::cin >> input;
    }
    else
        input = 1;
    if (input != 1) return 0;

    MissionState state = IDLE;
    ros::Time last_req = ros::Time::now();

    while (ros::ok()) {
        // -----------------------------------------------------------
        // [新增架构]：每帧统一构建、合并、刷新，杜绝重影与双重衰减
        // -----------------------------------------------------------
        if (flag_init_pos && static_walls.empty())
        {
            build_static_walls();
        }
        // 合并：PCL 抓到的真实柱子 + 焊死的虚拟围墙
        std::vector<Obstacle> all_obs = obstacles;
        if (!static_walls.empty())
        {
            all_obs.insert(all_obs.end(), static_walls.begin(), static_walls.end());
        }

        global_grid.update_with_memory(all_obs, cfg.uav_radius, cfg.safe_margin);
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

        switch (state) {
        case IDLE:
            if (mavros_connection_state.mode != "OFFBOARD" &&
                (ros::Time::now() - last_req > ros::Duration(5.0)))
            {
                mavros_msgs::SetMode srv;
                srv.request.custom_mode = "OFFBOARD";
                client_mode.call(srv);
                last_req = ros::Time::now();
            }
            else if (!mavros_connection_state.armed &&
                     (ros::Time::now() - last_req > ros::Duration(5.0)))
            {
                mavros_msgs::CommandBool srv;
                srv.request.value = true;
                client_arm.call(srv);
                last_req = ros::Time::now();
            }
            if (mavros_connection_state.armed) {
                if (!flag_init_pos) {
                    init_pos_x        = local_pos.pose.pose.position.x;
                    init_pos_y        = local_pos.pose.pose.position.y;
                    init_pos_z        = local_pos.pose.pose.position.z;
                    init_yaw_take_off = current_yaw;
                    flag_init_pos     = true;
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
            if (std::abs(local_pos.pose.pose.position.z - setpoint_raw.position.z) < 0.2) {
                state           = LEG1_AVOID;
                has_global_plan = false;
                ROS_INFO(">>> 避障前往 WP1");
            }
            break;

        case LEG1_AVOID:
            if (execute_avoidance_step({init_pos_x + cfg.wp1[0], init_pos_y + cfg.wp1[1]}, all_obs))
            {
                state = TURN1;
                ROS_INFO(">>> 转向右");
            }
            break;

        case TURN1: {
            float target_yaw       = init_yaw_take_off  - M_PI / 2.0;
            setpoint_raw.yaw       = calc_smooth_yaw(target_yaw, setpoint_raw.yaw, dt);
            setpoint_raw.type_mask = 0b101111111000;
            if (get_yaw_diff(target_yaw) < 0.1) {
                ros::Duration(1.0).sleep();
                state = LEG2_CROSS;
                ring_ctrl.reset();
                ROS_INFO(">>> 视觉穿门");
            }
            break;
        }

        case LEG2_CROSS: {
            float d2 = get_dist(cfg.wp2[0], cfg.wp2[1]);
            if (ring_ctrl.compute_cmd(local_pos, current_yaw, setpoint_raw) || d2 < 0.5) {
                state = RECOVER;
                ROS_INFO(">>> 恢复位置控制");
                setpoint_raw.type_mask        = 0b101111111000;
                setpoint_raw.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
                setpoint_raw.position.z       = init_pos_z + cfg.takeoff_height;
                setpoint_raw.position.x       = init_pos_x + cfg.wp2[0];
                setpoint_raw.position.y       = init_pos_y + cfg.wp2[1];
            }
            break;
        }

        case RECOVER:
            setpoint_raw.position.x = init_pos_x + cfg.wp2[0];
            setpoint_raw.position.y = init_pos_y + cfg.wp2[1];
            if (get_dist(cfg.wp2[0], cfg.wp2[1]) < 0.3) {
                state = TURN2;
                ROS_INFO(">>> 转向下");
            }
            break;

        case TURN2: {
            float target_yaw = init_yaw_take_off  + M_PI;
            setpoint_raw.yaw = calc_smooth_yaw(target_yaw, setpoint_raw.yaw, dt);
            if (get_yaw_diff(target_yaw) < 0.1) {
                state           = LEG3_AVOID;
                has_global_plan = false;
                ROS_INFO(">>> 避障前往 WP3");
            }
            break;
        }

        case LEG3_AVOID:
            if (execute_avoidance_step({init_pos_x + cfg.wp3[0], init_pos_y + cfg.wp3[1]}, all_obs))
            {
                state = TURN3;
                ROS_INFO(">>> 转向右");
            }
            break;

        case TURN3: {
            float target_yaw = init_yaw_take_off - M_PI / 2.0;
            setpoint_raw.yaw = calc_smooth_yaw(target_yaw, setpoint_raw.yaw, dt);
            if (get_yaw_diff(target_yaw) < 0.1) {
                state           = LEG4_FINAL;
                has_global_plan = false;
                ROS_INFO(">>> 前往终点");
            }
            break;
        }

        case LEG4_FINAL:
            if (execute_avoidance_step({init_pos_x + cfg.wp4[0], init_pos_y + cfg.wp4[1]}, all_obs))
            {
                state = LANDING_SEARCH;
                ROS_INFO(">>> 降落搜索");
                search_mode_dir = false;
                last_req = ros::Time::now();
            }
            break;

        case LANDING_SEARCH:
            mission_num_msg.data = 2;  // SEARCH/FOLLOW
            mission_num_pub.publish(mission_num_msg);

            {
                float scan_y            = init_pos_y + cfg.wp4[1];
                float scan_x_min        = init_pos_x + 0.0;
                float scan_x_max        = init_pos_x + 3.6;

                setpoint_raw.type_mask  = 0b101111100011;  // Vx, Vy, Z, Yaw
                setpoint_raw.velocity.x = satfunc(
                    (scan_y - local_pos.pose.pose.position.y) * cfg.p_xy, cfg.vel_track_max);
                setpoint_raw.position.z = init_pos_z + cfg.takeoff_height;
                setpoint_raw.yaw = init_yaw_take_off;

                if (!search_mode_dir) {
                    setpoint_raw.velocity.x =
                        satfunc((scan_x_max - local_pos.pose.pose.position.x) * cfg.p_xy,
                                cfg.vel_track_max);
                    if (std::abs(local_pos.pose.pose.position.x - scan_x_max) < 0.3)
                        search_mode_dir = true;
                }
                else {
                    setpoint_raw.velocity.x =
                        satfunc((scan_x_min - local_pos.pose.pose.position.x) * cfg.p_xy,
                                cfg.vel_track_max);
                    if (std::abs(local_pos.pose.pose.position.x - scan_x_min) < 0.3)
                        search_mode_dir = false;
                }
            }

            if (land_detected && takeoff_color == land_color && yolo_result.point.z > 0.5) {
                state = LANDING_FOLLOW;
                ROS_INFO(">>> 锁定目标，开始视觉伺服");
                last_req = ros::Time::now();
            }
            break;

        case LANDING_FOLLOW: {
            bool flag = false;
            if (flag) {
                setpoint_raw.position.z = local_pos.pose.pose.position.z - 0.15;
                if (local_pos.pose.pose.position.z < init_pos_z + 0.15) {
                    state = FINISHED;
                    ROS_INFO(">>> 任务完成");
                }
            }
            mission_num_msg.data = 2;
            mission_num_pub.publish(mission_num_msg);
            if (ros::Time::now() - last_req > ros::Duration(cfg.time_threshold) && !land_detected) {
                state = LANDING_SEARCH;
                ROS_WARN("目标丢失，重新搜索");
                break;
            }
            if (land_detected) last_req = ros::Time::now();
            {
                float vx = satfunc(yolo_result.point.y * cfg.yolo_follow_kp, cfg.vel_track_max);
                float vy = satfunc(yolo_result.point.x * cfg.yolo_follow_kp, cfg.vel_track_max);
                setpoint_raw.type_mask  = 0b100111000011;
                setpoint_raw.velocity.x = vx;
                setpoint_raw.velocity.y = vy;
                if (!flag) {
                    setpoint_raw.position.z = init_pos_z + cfg.takeoff_height;
                }
                if (std::hypot(yolo_result.point.x, yolo_result.point.y) < 0.1) {
                    flag = true;
                    ROS_INFO(">>> 对准，降落");
                }
            }
            break;
        }

            // case LANDING_DESCEND:
            //     mission_num_msg.data = 3;
            //     mission_num_pub.publish(mission_num_msg);
            //     setpoint_raw.type_mask  = 0b101111111000;
            //     setpoint_raw.position.x = local_pos.pose.pose.position.x;
            //     setpoint_raw.position.y = local_pos.pose.pose.position.y;
            //     setpoint_raw.position.z = local_pos.pose.pose.position.z - 0.15;
            //     if (local_pos.pose.pose.position.z < init_pos_z + 0.15) {
            //         state = FINISHED;
            //         ROS_INFO(">>> 任务完成");
            //     }
            //     break;

        case FINISHED: setpoint_raw.type_mask = 0; break;
        }

        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
