/**
 * @file astar.cpp
 * @brief 终极优化版：平滑转向、参数外置、抗震荡、支持方柱矩形精确膨胀避障
 */
#include "astar.h"
#include "ring_crossing.h"
#include <iostream>
#include <algorithm>
#include <cmath>
#include <clocale>
#include <tf/transform_listener.h>

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

std::vector<Obstacle> obstacles;
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
tf::TransformListener *tf_listener = nullptr;

// Scan_Land
std::string takeoff_color = "";
std::string land_color = "";
bool land_detected = false;
geometry_msgs::PointStamped yolo_result;
std_msgs::Int8 mission_num_msg;
bool search_mode_dir = false;

// VFH 上一帧角度缓存 (用于滤波)
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
  {
    return target_yaw;
  }
  else
  {
    return current_yaw + (diff > 0 ? max_step : -max_step);
  }
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

// ============================================================================
// 4. 感知模块
// ============================================================================
void detection_cb_wrapper(const pcl_detection::ObjectDetectionResult::ConstPtr &msg)
{
  if (!flag_init_pos)
    return;
  if (!msg->success)
    return;

  obstacles.clear();
  Eigen::Vector2f drone_p(local_pos.pose.pose.position.x, local_pos.pose.pose.position.y);

  int valid_cnt = 0;
  for (const auto &obj : msg->objects)
  {
    if (!std::isfinite(obj.position.x) || !std::isfinite(obj.position.y))
      continue;

    float wx = obj.position.x;
    float wy = obj.position.y;
    float dist_rel = std::hypot(wx - drone_p.x(), wy - drone_p.y());

    // 过滤太远或太近的物体
    if (dist_rel > 10.0f || dist_rel < 0.1f)
      continue;

    Obstacle obs;
    obs.id = 0;
    obs.type = obj.type;
    obs.position = Eigen::Vector2f(wx, wy);

    if (obs.type == WALL) // 类型 0
    {
      obs.radius = cfg.wall_radius;
      obs.width = obj.width;
      obs.length = obj.width; // 墙的长度取 width

      // [核心修复] 根据 PCL 传来的平面方程(法向量)，计算墙体真实的偏转角度
      if (obj.plane_coeffs.size() >= 4)
      {
        float A = obj.plane_coeffs[0];
        float B = obj.plane_coeffs[1];
        // 法向量在 XY 平面的投影是 (A, B)，墙面走向垂直于法向量，即 (-B, A)
        obs.angle = std::atan2(A, -B);
      }
      else
      {
        obs.angle = 0; // 降级处理
      }
      valid_cnt++;
      obstacles.push_back(obs);
    }
    else if (obs.type == RING) // 类型 3：环门，不需要建入障碍物地图
    {
      continue;
    }
    else if (obs.type == PILLAR) // 类型 4：方柱
    {
      obs.width = obj.width;
      obs.length = obj.height; // OBB 深度
      obs.radius = 0;
      obs.angle = 0;
      valid_cnt++;
      obstacles.push_back(obs);
    }
  }

  // [诊断雷达]：只要节点接通了，终端每秒都会刷出这句话。如果一直不印，说明依然是话题或MD5问题。
  ROS_INFO_THROTTLE(1.0, "[A* 避障节点] 收到 PCL 数据: 包含 %zu 个物体, 成功解析绘制 %d 个", msg->objects.size(), valid_cnt);
}

// ------------------ 地图与规划核心函数 ------------------
OccupancyGrid2D::OccupancyGrid2D()
{
  resolution = 0.1f;
  origin_x = -10.0f;
  origin_y = -10.0f;
  for (int i = 0; i < GRID_W; ++i)
    for (int j = 0; j < GRID_H; ++j)
      cells[i][j] = 0;
}
bool OccupancyGrid2D::world_to_grid(float wx, float wy, int &gx, int &gy) const
{
  gx = (int)((wx - origin_x) / resolution);
  gy = (int)((wy - origin_y) / resolution);
  return (gx >= 0 && gx < GRID_W && gy >= 0 && gy < GRID_H);
}
void OccupancyGrid2D::grid_to_world(int gx, int gy, float &wx, float &wy) const
{
  wx = origin_x + (gx + 0.5f) * resolution;
  wy = origin_y + (gy + 0.5f) * resolution;
}
bool OccupancyGrid2D::is_occupied(int gx, int gy) const
{
  if (gx < 0 || gx >= GRID_W || gy < 0 || gy >= GRID_H)
    return true;
  return cells[gx][gy] > OBS_THRESHOLD;
}

void OccupancyGrid2D::update_with_memory(const std::vector<Obstacle> &obstacles, float drone_r, float safe_margin)
{
  Eigen::Vector2f drone_p(local_pos.pose.pose.position.x, local_pos.pose.pose.position.y);
  bool is_fast_turning = std::abs(current_yaw_rate) > cfg.rotation_gating_threshold;

  // [诊断雷达]：如果你发现地图画不出来，看看是不是这里一直报错！
  if (is_fast_turning)
  {
    ROS_WARN_THROTTLE(2.0, "[A* 建图警告] 无人机角速度过大 (%.2f)，为防重影已暂停建图！", current_yaw_rate);
  }

  // 地图记忆衰减逻辑 (保持不变)
  for (int i = 0; i < GRID_W; ++i)
  {
    for (int j = 0; j < GRID_H; ++j)
    {
      if (cells[i][j] > 0)
      {
        float wx, wy;
        grid_to_world(i, j, wx, wy);
        float dist = std::hypot(wx - drone_p.x(), wy - drone_p.y());
        int decay = 0;
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
  if (is_fast_turning)
    return;

  float total_margin = drone_r + safe_margin;
  for (const auto &obs : obstacles)
  {
    if ((obs.position - drone_p).norm() < 0.2f)
      continue;

    if (obs.type == PILLAR)
    {
      float min_x = obs.position.x() - obs.width / 2.0f - total_margin;
      float max_x = obs.position.x() + obs.width / 2.0f + total_margin;
      float min_y = obs.position.y() - obs.length / 2.0f - total_margin;
      float max_y = obs.position.y() + obs.length / 2.0f + total_margin;

      int min_gx, min_gy, max_gx, max_gy;
      world_to_grid(min_x, min_y, min_gx, min_gy);
      world_to_grid(max_x, max_y, max_gx, max_gy);

      min_gx = std::max(0, min_gx);
      min_gy = std::max(0, min_gy);
      max_gx = std::min(GRID_W - 1, max_gx);
      max_gy = std::min(GRID_H - 1, max_gy);

      for (int x = min_gx; x <= max_gx; ++x)
      {
        for (int y = min_gy; y <= max_gy; ++y)
        {
          cells[x][y] = MAX_HEALTH;
        }
      }
    }
    else if (obs.type == WALL)
    {
      float hl = obs.length / 2.0f;
      Eigen::Vector2f dir(cos(obs.angle), sin(obs.angle));
      Eigen::Vector2f p1 = obs.position - dir * hl;
      Eigen::Vector2f p2 = obs.position + dir * hl;
      float exp = obs.radius + total_margin;
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
      max_gx = std::min(GRID_W - 1, max_gx);
      max_gy = std::min(GRID_H - 1, max_gy);
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
}

bool run_astar(const OccupancyGrid2D &grid, Eigen::Vector2f start, Eigen::Vector2f goal, std::vector<Eigen::Vector2f> &out_path)
{
  out_path.clear();
  int sgx, sgy, ggx, ggy;
  if (!grid.world_to_grid(start.x(), start.y(), sgx, sgy) || !grid.world_to_grid(goal.x(), goal.y(), ggx, ggy))
    return false;
  auto find_free = [&](int &cx, int &cy) -> bool
  {if(!grid.is_occupied(cx,cy))return true;std::queue<std::pair<int,int>>q;q.push({cx,cy});bool vis[200][200]={false};vis[cx][cy]=true;int s=500;while(!q.empty()&&s--){auto cur=q.front();q.pop();if(!grid.is_occupied(cur.first,cur.second)){cx=cur.first;cy=cur.second;return true;}int dx[]={1,-1,0,0},dy[]={0,0,1,-1};for(int i=0;i<4;++i){int nx=cur.first+dx[i],ny=cur.second+dy[i];if(nx>=0&&nx<200&&ny>=0&&ny<200&&!vis[nx][ny]){vis[nx][ny]=true;q.push({nx,ny});}}}return false; };
  if (grid.is_occupied(sgx, sgy))
    find_free(sgx, sgy);
  if (grid.is_occupied(ggx, ggy))
    find_free(ggx, ggy);
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
    int cx = curr / 200, cy = curr % 200;
    int dx[] = {1, -1, 0, 0, 1, 1, -1, -1}, dy[] = {0, 0, 1, -1, 1, -1, 1, -1};
    float dists[] = {1, 1, 1, 1, 1.4, 1.4, 1.4, 1.4};
    for (int i = 0; i < 8; ++i)
    {
      int nx = cx + dx[i], ny = cy + dy[i];
      if (nx < 0 || nx >= 200 || ny < 0 || ny >= 200 || grid.is_occupied(nx, ny))
        continue;
      int nid = nx * 200 + ny;
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
    grid.grid_to_world(curr / 200, curr % 200, wx, wy);
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
  std::vector<Eigen::Vector2f> result;
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
      result.push_back(b0 * pts[i] + b1 * pts[i + 1] + b2 * pts[i + 2] + b3 * pts[i + 3]);
    }
  }
  result.push_back(cps.back());
  return result;
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

// ------------------ VFH+ 优化 (完美支持方柱视场阻挡) ------------------
bool run_vfh_plus(Eigen::Vector2f target, const std::vector<Obstacle> &obs, bool &need_replan)
{
  need_replan = false;
  Eigen::Vector2f curr(local_pos.pose.pose.position.x, local_pos.pose.pose.position.y);
  Eigen::Vector2f dir = target - curr;
  float dist = dir.norm();
  if (dist < 0.2)
    return true;

  if (vfh_first_run)
  {
    last_vfh_yaw = current_yaw;
    vfh_first_run = false;
  }

  const int BINS = 72;
  float hist[BINS] = {0};
  float min_obs_d = 1e9;

  for (const auto &o : obs)
  {
    if (o.type == RING)
      continue;

    if (o.type == PILLAR)
    {
      // [核心修改] VFH矩形精确视场遮挡（FOV block），不膨胀圆
      float total_margin = cfg.uav_radius + cfg.safe_margin;
      float min_x = o.position.x() - o.width / 2.0f - total_margin;
      float max_x = o.position.x() + o.width / 2.0f + total_margin;
      float min_y = o.position.y() - o.length / 2.0f - total_margin;
      float max_y = o.position.y() + o.length / 2.0f + total_margin;

      // 计算无人机到该膨胀后矩形框的最短距离
      float dx = std::max({min_x - curr.x(), 0.0f, curr.x() - max_x});
      float dy = std::max({min_y - curr.y(), 0.0f, curr.y() - max_y});
      float d = std::hypot(dx, dy);

      if (d < min_obs_d)
        min_obs_d = d;
      if (d > 3.0 || d < 0.1)
        continue;

      // 提取四个角，算它们针对无人机当前视角的偏差
      Eigen::Vector2f corners[4] = {{min_x, min_y}, {min_x, max_y}, {max_x, min_y}, {max_x, max_y}};
      std::vector<float> angles;
      for (int i = 0; i < 4; ++i)
      {
        float ang = std::atan2(corners[i].y() - curr.y(), corners[i].x() - curr.x()) - current_yaw;
        while (ang > M_PI)
          ang -= 2 * M_PI;
        while (ang < -M_PI)
          ang += 2 * M_PI;
        angles.push_back(ang);
      }

      // 通过寻找最大缝隙(Max Gap)解决 -PI 到 PI 的跳变问题，锁定阻挡视场的真实起点和终点
      std::sort(angles.begin(), angles.end());
      float max_gap = angles[0] + 2 * M_PI - angles[3];
      int gap_idx = 3;
      for (int i = 0; i < 3; ++i)
      {
        float gap = angles[i + 1] - angles[i];
        if (gap > max_gap)
        {
          max_gap = gap;
          gap_idx = i;
        }
      }

      float start_ang, end_ang;
      if (gap_idx != 3)
      {
        start_ang = angles[gap_idx + 1];
        end_ang = angles[gap_idx] + 2 * M_PI; // 将终点跨越映射为连贯的角度
      }
      else
      {
        start_ang = angles[0];
        end_ang = angles[3];
      }

      // 阻塞被方柱完全覆盖的扇区直方图
      int steps = std::ceil((end_ang - start_ang) / (2 * M_PI / BINS));
      for (int k = 0; k <= steps; ++k)
      {
        float a = start_ang + k * (2 * M_PI / BINS);
        int idx = (int)((a + M_PI) / (2 * M_PI) * BINS) % BINS;
        if (idx < 0)
          idx += BINS;
        hist[idx] += 10.0f / (d + 0.1f);
      }
    }
    else if (o.type == WALL)
    {
      Eigen::Vector2f to_obs = o.position - curr;
      float d = to_obs.norm();
      if (d < min_obs_d)
        min_obs_d = d;
      if (d > 3.0 || d < 0.1)
        continue;
      float angle = std::atan2(to_obs.y(), to_obs.x()) - current_yaw;
      while (angle > M_PI)
        angle -= 2 * M_PI;
      while (angle < -M_PI)
        angle += 2 * M_PI;
      float w_ang = std::asin(std::min(1.0f, (o.radius + cfg.uav_radius + cfg.safe_margin) / d));
      int c_idx = (int)((angle + M_PI) / (2 * M_PI) * BINS) % BINS;
      int hw = (int)(w_ang / (2 * M_PI) * BINS) + 1;
      for (int k = c_idx - hw; k <= c_idx + hw; ++k)
        hist[(k + BINS) % BINS] += 10.0f / d;
    }
  }

  if (min_obs_d < cfg.min_safe_dist)
  {
    need_replan = true;
    return false;
  }

  float t_yaw = std::atan2(dir.y(), dir.x());
  float rel_t_yaw = t_yaw - current_yaw;
  while (rel_t_yaw > M_PI)
    rel_t_yaw -= 2 * M_PI;
  while (rel_t_yaw < -M_PI)
    rel_t_yaw += 2 * M_PI;

  int best_idx = -1;
  float min_c = 1e9;
  for (int i = 0; i < BINS; ++i)
  {
    if (hist[i] > 15.0)
      continue;
    float b_yaw = -M_PI + i * (2 * M_PI / BINS) + (M_PI / BINS) * 0.5f;
    float diff_last = std::abs(b_yaw - (last_vfh_yaw - current_yaw));
    while (diff_last > M_PI)
      diff_last -= 2 * M_PI;
    float c = std::abs(b_yaw - rel_t_yaw) + hist[i] * 0.1f + std::abs(diff_last) * 0.5f;
    if (c < min_c)
    {
      min_c = c;
      best_idx = i;
    }
  }

  if (best_idx == -1)
  {
    ROS_WARN_THROTTLE(1.0, "[VFH] 局部死锁！");
    need_replan = true;
    return false;
  }

  float selected_yaw = -M_PI + best_idx * (2 * M_PI / BINS) + (M_PI / BINS) * 0.5f + current_yaw;
  float diff = selected_yaw - last_vfh_yaw;
  while (diff > M_PI)
    diff -= 2 * M_PI;
  while (diff < -M_PI)
    diff += 2 * M_PI;

  float final_yaw = last_vfh_yaw + diff * cfg.yaw_smooth_weight;
  last_vfh_yaw = final_yaw;

  pub_viz_vfh_vectors(t_yaw, final_yaw, curr);

  float speed = std::min(cfg.max_speed, dist);
  if (std::abs(diff) > 0.5)
    speed *= 0.3;

  setpoint_raw.position.x = curr.x() + std::cos(final_yaw) * speed * 0.5;
  setpoint_raw.position.y = curr.y() + std::sin(final_yaw) * speed * 0.5;
  setpoint_raw.position.z = init_pos_z + cfg.takeoff_height;
  setpoint_raw.yaw = final_yaw;
  return false;
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
void pub_viz_vfh_vectors(float t_yaw, float s_yaw, const Eigen::Vector2f &pos)
{
  visualization_msgs::Marker m;
  m.header.frame_id = "map";
  m.ns = "vfh";
  m.id = 0;
  m.type = 0;
  m.action = 0;
  m.pose.position.x = pos.x();
  m.pose.position.y = pos.y();
  m.pose.position.z = init_pos_z + cfg.takeoff_height;
  m.scale.x = 1.0;
  m.scale.y = 0.05;
  m.scale.z = 0.05;
  m.pose.orientation.w = 1;
  m.color.r = 1;
  m.color.a = 1;
  tf::quaternionTFToMsg(tf::createQuaternionFromYaw(t_yaw), m.pose.orientation);
  pub_viz_vfh.publish(m);
  m.id = 1;
  m.color.r = 0;
  m.color.g = 1;
  tf::quaternionTFToMsg(tf::createQuaternionFromYaw(s_yaw), m.pose.orientation);
  pub_viz_vfh.publish(m);
}
void pub_viz_grid_map(const OccupancyGrid2D &grid)
{
  nav_msgs::OccupancyGrid msg;
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = "map";
  msg.info.resolution = grid.resolution;
  msg.info.width = 200;
  msg.info.height = 200;
  msg.info.origin.position.x = grid.origin_x;
  msg.info.origin.position.y = grid.origin_y;
  msg.info.origin.orientation.w = 1;
  msg.data.resize(40000);
  for (int i = 0; i < 40000; ++i)
    msg.data[i] = (int8_t)((grid.cells[i % 200][i / 200] > 100) ? 100 : grid.cells[i % 200][i / 200]);
  pub_viz_map.publish(msg);
}

// ============================================================================
// 逻辑封装：执行单步避障
// ============================================================================
bool execute_avoidance_step(Eigen::Vector2f goal)
{
  Eigen::Vector2f curr(local_pos.pose.pose.position.x, local_pos.pose.pose.position.y);
  global_grid.update_with_memory(obstacles, cfg.uav_radius, cfg.safe_margin);

  static int cnt = 0;
  if (cnt++ % 5 == 0)
    pub_viz_grid_map(global_grid);

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
    if (cnt % 5 == 0)
    {
      pub_viz_astar_path(global_path_raw);
      pub_viz_smooth_path(global_path_smooth);
    }

    Eigen::Vector2f la = get_lookahead_point(global_path_smooth, curr, cfg.lookahead_dist);
    bool stuck = false;
    bool reached = run_vfh_plus(la, obstacles, stuck);

    if (stuck)
      has_global_plan = false;
    if ((curr - goal).norm() < 0.3)
      return true;
  }
  return false;
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

  // tf_listener 保留定义避免外部依赖崩溃，但核心感知已不再使用
  tf_listener = new tf::TransformListener();
  ros::Duration(0.5).sleep();

  // 必须改回 /pcl_detection/result，因为你的 launch/yaml 里配的就是这个
  ros::Subscriber s1 = public_nh.subscribe("mavros/state", 10, state_cb);
  ros::Subscriber s2 = public_nh.subscribe("/mavros/local_position/odom", 10, local_pos_cb);
  ros::Subscriber s3 = public_nh.subscribe("/pcl_detection/result", 10, detection_cb_wrapper);
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
    global_grid.update_with_memory(obstacles, cfg.uav_radius, cfg.safe_margin);
    static int map_pub_cnt = 0;
    if (map_pub_cnt++ % 5 == 0)
      pub_viz_grid_map(global_grid);

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
      if (std::abs(local_pos.pose.pose.position.z - setpoint_raw.position.z) < 0.2)
      {
        state = LEG1_AVOID;
        has_global_plan = false;
        ROS_INFO(">>> 避障前往 WP1");
      }
      break;

    case LEG1_AVOID:
      if (execute_avoidance_step({init_pos_x + cfg.wp1[0], init_pos_y + cfg.wp1[1]}))
      {
        state = TURN1;
        ROS_INFO(">>> 转向右");
      }
      break;

    case TURN1:
    {
      float target_yaw = init_yaw_take_off + M_PI / 2.0 - M_PI / 2.0;
      setpoint_raw.yaw = calc_smooth_yaw(target_yaw, setpoint_raw.yaw, dt);
      setpoint_raw.type_mask = 0b101111111000;
      if (get_yaw_diff(target_yaw) < 0.1)
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
        ROS_INFO(">>> 恢复位置控制");
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
        ROS_INFO(">>> 转向下");
      }
      break;

    case TURN2:
    {
      float target_yaw = init_yaw_take_off + M_PI / 2.0 + M_PI;
      setpoint_raw.yaw = calc_smooth_yaw(target_yaw, setpoint_raw.yaw, dt);
      if (get_yaw_diff(target_yaw) < 0.1)
      {
        state = LEG3_AVOID;
        has_global_plan = false;
        ROS_INFO(">>> 避障前往 WP3");
      }
      break;
    }

    case LEG3_AVOID:
      if (execute_avoidance_step({init_pos_x + cfg.wp3[0], init_pos_y + cfg.wp3[1]}))
      {
        state = TURN3;
        ROS_INFO(">>> 转向右");
      }
      break;

    case TURN3:
    {
      float target_yaw = init_yaw_take_off + M_PI / 2.0 - M_PI / 2.0;
      setpoint_raw.yaw = calc_smooth_yaw(target_yaw, setpoint_raw.yaw, dt);
      if (get_yaw_diff(target_yaw) < 0.1)
      {
        state = LEG4_FINAL;
        has_global_plan = false;
        ROS_INFO(">>> 前往终点");
      }
      break;
    }

    case LEG4_FINAL:
      if (execute_avoidance_step({init_pos_x + cfg.wp4[0], init_pos_y + cfg.wp4[1]}))
      {
        state = LANDING_SEARCH;
        ROS_INFO(">>> 降落搜索");
        search_mode_dir = false;
        last_req = ros::Time::now();
      }
      break;

    case LANDING_SEARCH:
      mission_num_msg.data = 2;
      mission_num_pub.publish(mission_num_msg);
      {
        float scan_x = init_pos_x + cfg.wp4[0];
        float scan_y_min = init_pos_y + cfg.wp4[1];
        float scan_y_max = init_pos_y + cfg.wp4[1] + 4.0;
        setpoint_raw.type_mask = 0b101111111000;
        setpoint_raw.position.x = scan_x;
        setpoint_raw.position.z = init_pos_z + cfg.takeoff_height;
        setpoint_raw.yaw = calc_smooth_yaw(init_yaw_take_off - M_PI / 2.0, setpoint_raw.yaw, dt);
        if (!search_mode_dir)
        {
          setpoint_raw.position.y = scan_y_max;
          if (std::abs(local_pos.pose.pose.position.y - scan_y_max) < 0.3)
            search_mode_dir = true;
        }
        else
        {
          setpoint_raw.position.y = scan_y_min;
          if (std::abs(local_pos.pose.pose.position.y - scan_y_min) < 0.3)
            search_mode_dir = false;
        }
      }
      if (land_detected && takeoff_color == land_color && yolo_result.point.z > 0.5)
      {
        state = LANDING_FOLLOW;
        ROS_INFO(">>> 锁定目标，开始视觉伺服");
        last_req = ros::Time::now();
      }
      break;

    case LANDING_FOLLOW:
      mission_num_msg.data = 2;
      mission_num_pub.publish(mission_num_msg);
      if (ros::Time::now() - last_req > ros::Duration(cfg.time_threshold) && !land_detected)
      {
        state = LANDING_SEARCH;
        ROS_WARN("目标丢失，重新搜索");
        break;
      }
      if (land_detected)
        last_req = ros::Time::now();
      {
        float vx = satfunc(yolo_result.point.y * cfg.yolo_follow_kp, cfg.vel_track_max);
        float vy = satfunc(yolo_result.point.x * cfg.yolo_follow_kp, cfg.vel_track_max);
        setpoint_raw.type_mask = 0b100111000011;
        setpoint_raw.velocity.x = vx;
        setpoint_raw.velocity.y = vy;
        setpoint_raw.position.z = init_pos_z + cfg.takeoff_height;
        if (std::hypot(yolo_result.point.x, yolo_result.point.y) < 0.1)
        {
          state = LANDING_DESCEND;
          ROS_INFO(">>> 对准，降落");
        }
      }
      break;

    case LANDING_DESCEND:
      mission_num_msg.data = 3;
      mission_num_pub.publish(mission_num_msg);
      setpoint_raw.type_mask = 0b101111111000;
      setpoint_raw.position.x = local_pos.pose.pose.position.x;
      setpoint_raw.position.y = local_pos.pose.pose.position.y;
      setpoint_raw.position.z = local_pos.pose.pose.position.z - 0.15;
      if (local_pos.pose.pose.position.z < init_pos_z + 0.15)
      {
        state = FINISHED;
        ROS_INFO(">>> 任务完成");
      }
      break;

    case FINISHED:
      setpoint_raw.type_mask = 0;
      break;
    }

    ros::spinOnce();
    rate.sleep();
  }
  return 0;
}