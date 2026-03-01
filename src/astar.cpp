/**
 * @file astar.cpp
 * @brief 最终修复版：引入 TF 解决 5秒 延时导致的坐标漂移问题
 */
#include "astar.h"
#include "ring_crossing.h"
#include <iostream>
#include <algorithm>
#include <cmath>
#include <clocale>
#include <tf/transform_listener.h> // 必须引入

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
tf::Quaternion quat;

float init_pos_x = 0, init_pos_y = 0, init_pos_z = 0;
bool flag_init_pos = false;

std::vector<Eigen::Vector2f> global_path_raw;
std::vector<Eigen::Vector2f> global_path_smooth;
ros::Time last_replan_time;
bool has_global_plan = false;

OccupancyGrid2D global_grid;

// === 新增：TF 监听器指针 ===
tf::TransformListener *tf_listener = nullptr;

// ============================================================================
// 2. 配置参数
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
  float check_radius;
  int map_decay_rate;
} cfg;

ros::Publisher pub_setpoint;
ros::Publisher pub_viz_path_raw;
ros::Publisher pub_viz_path_smooth;
ros::Publisher pub_viz_vfh;
ros::Publisher pub_viz_map;

// ============================================================================
// 3. 辅助函数
// ============================================================================
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

  cfg.check_radius = cfg.uav_radius + 0.1f;
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
// 4. 感知回调 (核心修复：TF 历史坐标变换)
// ============================================================================
void detection_cb_wrapper(const pcl_detection::ObjectDetectionResult::ConstPtr &msg)
{
  if (!flag_init_pos || !tf_listener)
    return;

  obstacles.clear();
  Eigen::Vector2f drone_p(local_pos.pose.pose.position.x, local_pos.pose.pose.position.y);

  // 目标坐标系：map (也就是 MAVROS 的 local_origin)
  std::string target_frame = "map";

  for (const auto &obj : msg->objects)
  {
    if (!std::isfinite(obj.position.x) || !std::isfinite(obj.position.y))
      continue;

    // 1. 构建点 (带时间戳)
    geometry_msgs::PointStamped pt_in, pt_out;
    pt_in.header = msg->header; // 使用 PCL 消息里的历史时间戳！
    pt_in.point.x = obj.position.x;
    pt_in.point.y = obj.position.y;
    pt_in.point.z = obj.position.z;

    try
    {
      // 2. 核心：请求 TF 进行历史变换
      // 等待变换关系可用 (最多等 0.1s)
      // 注意：如果 msg->header.frame_id 是空的，默认设为 "base_link" (视你的雷达挂载而定)
      if (pt_in.header.frame_id.empty())
        pt_in.header.frame_id = "base_link";

      // 执行变换：把 "几秒前" 在 "雷达系" 下的点，转换到 "map系" 下
      tf_listener->transformPoint(target_frame, pt_in, pt_out);
    }
    catch (tf::TransformException &ex)
    {
      ROS_WARN_THROTTLE(1.0, "[TF Error] 无法转换坐标: %s. 使用当前位姿硬算...", ex.what());
      // 降级处理：还是用手算 (虽然会有误差，总比没有好)
      float cos_y = cos(current_yaw), sin_y = sin(current_yaw);
      pt_out.point.x = drone_p.x() + obj.position.x * cos_y - obj.position.y * sin_y;
      pt_out.point.y = drone_p.y() + obj.position.x * sin_y + obj.position.y * cos_y;
    }

    float wx = pt_out.point.x;
    float wy = pt_out.point.y;

    // 过滤
    float dist_rel = std::hypot(wx - drone_p.x(), wy - drone_p.y());
    if (dist_rel > 10.0f || dist_rel < 0.5f)
      continue;

    Obstacle obs;
    obs.id = 0;
    obs.type = (obj.type == 0) ? WALL : CYLINDER;
    obs.position = Eigen::Vector2f(wx, wy);

    if (obs.type == WALL)
    {
      // 墙体法向量也需要旋转。这里因为是方向向量，只受旋转影响
      // 简单起见，我们假设墙体在世界系下是静止的，
      // 实际上墙体角度转换也需要 TF，这里简化处理，直接用 geometric 方法
      // 如果需要精确，应该把 (nx, ny, 0) 当作 Vector3Stamped 进行 transformVector

      double nx = obj.plane_coeffs[0];
      double ny = obj.plane_coeffs[1];

      // 尝试对法向量进行 TF 变换 (Vector 而不是 Point)
      geometry_msgs::Vector3Stamped vec_in, vec_out;
      vec_in.header = msg->header;
      if (vec_in.header.frame_id.empty())
        vec_in.header.frame_id = "base_link";
      vec_in.vector.x = nx;
      vec_in.vector.y = ny;
      vec_in.vector.z = 0;

      try
      {
        tf_listener->transformVector(target_frame, vec_in, vec_out);
        nx = vec_out.vector.x;
        ny = vec_out.vector.y;
      }
      catch (...)
      { /* Fallback using current yaw */
        float cy = cos(current_yaw), sy = sin(current_yaw);
        float tmp_nx = nx;
        nx = tmp_nx * cy - ny * sy;
        ny = tmp_nx * sy + ny * cy;
      }

      obs.radius = 0.05f;
      obs.length = obj.width;
      // 切线方向 = 法向量旋转90度
      obs.angle = std::atan2(nx, -ny);
    }
    else
    {
      if (obj.radius > 3.0f)
        continue;
      obs.radius = obj.radius;
      obs.length = 0;
      obs.angle = 0;
    }
    obstacles.push_back(obs);
  }
}

// ============================================================================
// 地图方法
// ============================================================================
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

  // 三区衰减
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
          decay = 10;
        else if (dist > 20.0f)
          decay = 2;
        cells[i][j] = std::max(0, cells[i][j] - decay);
      }
    }
  }

  float total_margin = drone_r + safe_margin;
  for (const auto &obs : obstacles)
  {
    if ((obs.position - drone_p).norm() < drone_r + 0.1f)
      continue;

    if (obs.type == CYLINDER)
    {
      int gx, gy;
      if (!world_to_grid(obs.position.x(), obs.position.y(), gx, gy))
        continue;
      float effect_r = obs.radius + total_margin;
      int r_cells = std::ceil(effect_r / resolution);
      float r_sq = pow(effect_r / resolution, 2);

      for (int dx = -r_cells; dx <= r_cells; ++dx)
      {
        for (int dy = -r_cells; dy <= r_cells; ++dy)
        {
          int nx = gx + dx, ny = gy + dy;
          if (nx >= 0 && nx < GRID_W && ny >= 0 && ny < GRID_H)
          {
            if (dx * dx + dy * dy <= r_sq)
              cells[nx][ny] = MAX_HEALTH;
          }
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
          {
            cells[x][y] = MAX_HEALTH;
          }
        }
      }
    }
  }
}

// ... [此处省略 run_astar, is_path_blocked, get_lookahead_point, run_vfh_plus, generate_smooth_path, viz 等函数] ...
// ... [请直接复用上一版 src/astar.cpp 中这些完全未变动的算法实现] ...
// ... [务必确保这些函数体存在] ...
// ------------------ 占位符 START ------------------
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
    bool vis[200][200] = {false};
    vis[cx][cy] = true;
    int steps = 500;
    while (!q.empty() && steps--)
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
        if (nx >= 0 && nx < 200 && ny >= 0 && ny < 200 && !vis[nx][ny])
        {
          vis[nx][ny] = true;
          q.push({nx, ny});
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
    float dists[] = {1, 1, 1, 1, 1.414, 1.414, 1.414, 1.414};
    for (int i = 0; i < 8; ++i)
    {
      int nx = cx + dx[i], ny = cy + dy[i];
      if (nx < 0 || nx >= 200 || ny < 0 || ny >= 200)
        continue;
      if (grid.is_occupied(nx, ny))
        continue;
      int n_id = nx * 200 + ny;
      float new_g = g_cost[curr] + dists[i];
      if (new_g < g_cost[n_id])
      {
        g_cost[n_id] = new_g;
        parent[n_id] = curr;
        float h = std::hypot(nx - ggx, ny - ggy) * cfg.astar_weight;
        open.push({new_g + h, n_id});
      }
    }
  }
  if (!found)
    return false;
  std::vector<Eigen::Vector2f> raw_pts;
  int curr = goal_id;
  while (curr != -1)
  {
    float wx, wy;
    grid.grid_to_world(curr / 200, curr % 200, wx, wy);
    raw_pts.push_back({wx, wy});
    curr = parent[curr];
  }
  std::reverse(raw_pts.begin(), raw_pts.end());
  if (!raw_pts.empty())
  {
    out_path.push_back(raw_pts[0]);
    for (size_t i = 1; i < raw_pts.size() - 1; ++i)
      if ((raw_pts[i] - out_path.back()).norm() > 0.6)
        out_path.push_back(raw_pts[i]);
    out_path.push_back(raw_pts.back());
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
      float u = (float)j / (float)points_per_seg;
      float b0 = (1 - u) * (1 - u) * (1 - u) / 6.0f, b1 = (3 * u * u * u - 6 * u * u + 4) / 6.0f, b2 = (-3 * u * u * u + 3 * u * u + 3 * u + 1) / 6.0f, b3 = u * u * u / 6.0f;
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
  Eigen::Vector2f drone_pos(local_pos.pose.pose.position.x, local_pos.pose.pose.position.y);
  int start_idx = 0;
  float min_dist = 1e9;
  for (int i = 0; i < path.size(); ++i)
  {
    float d = (path[i] - drone_pos).norm();
    if (d < min_dist)
    {
      min_dist = d;
      start_idx = i;
    }
  }
  for (int i = start_idx; i < path.size(); ++i)
  {
    if ((path[i] - drone_pos).norm() > 6.0)
      break;
    int gx, gy;
    if (grid.world_to_grid(path[i].x(), path[i].y(), gx, gy))
    {
      if (grid.cells[gx][gy] > 900)
        return true;
    }
  }
  return false;
}

Eigen::Vector2f get_lookahead_point(const std::vector<Eigen::Vector2f> &path, Eigen::Vector2f curr_pos, float lookahead_dist)
{
  if (path.empty())
    return curr_pos;
  float min_dist = 1e9;
  int idx = 0;
  for (int i = 0; i < path.size(); ++i)
  {
    float d = (path[i] - curr_pos).norm();
    if (d < min_dist)
    {
      min_dist = d;
      idx = i;
    }
  }
  float dist_acc = 0;
  for (int i = idx; i < path.size() - 1; ++i)
  {
    float seg_len = (path[i + 1] - path[i]).norm();
    if (dist_acc + seg_len > lookahead_dist)
    {
      float ratio = (lookahead_dist - dist_acc) / seg_len;
      return path[i] + (path[i + 1] - path[i]) * ratio;
    }
    dist_acc += seg_len;
  }
  return path.back();
}

bool run_vfh_plus(Eigen::Vector2f target, const std::vector<Obstacle> &obs, bool &need_replan)
{
  need_replan = false;
  Eigen::Vector2f curr(local_pos.pose.pose.position.x, local_pos.pose.pose.position.y);
  Eigen::Vector2f dir = target - curr;
  float dist = dir.norm();
  if (dist < 0.2)
    return true;
  const int BINS = 72;
  float hist[BINS] = {0};
  for (const auto &o : obs)
  {
    Eigen::Vector2f to_obs = o.position - curr;
    float d = to_obs.norm();
    if (d > 4.5 || d < 0.1)
      continue;
    float angle = std::atan2(to_obs.y(), to_obs.x()) - current_yaw;
    while (angle > M_PI)
      angle -= 2 * M_PI;
    while (angle < -M_PI)
      angle += 2 * M_PI;
    float effect_r = o.radius + cfg.uav_radius + cfg.safe_margin;
    float val = effect_r / d;
    if (val > 1.0f)
      val = 1.0f;
    float width_ang = std::asin(val);
    int center_idx = (int)((angle + M_PI) / (2 * M_PI) * BINS) % BINS;
    int half_width = (int)(width_ang / (2 * M_PI) * BINS) + 1;
    for (int k = center_idx - half_width; k <= center_idx + half_width; ++k)
    {
      int idx = (k + BINS) % BINS;
      hist[idx] += 10.0f / d;
    }
  }
  float target_yaw = std::atan2(dir.y(), dir.x());
  float rel_target_yaw = target_yaw - current_yaw;
  while (rel_target_yaw > M_PI)
    rel_target_yaw -= 2 * M_PI;
  while (rel_target_yaw < -M_PI)
    rel_target_yaw += 2 * M_PI;
  int best_idx = -1;
  float min_cost = 1e9;
  for (int i = 0; i < BINS; ++i)
  {
    if (hist[i] > 15.0)
      continue;
    float bin_yaw = -M_PI + i * (2 * M_PI / BINS) + (M_PI / BINS) * 0.5f;
    float cost = std::abs(bin_yaw - rel_target_yaw) + hist[i] * 0.1f;
    if (cost < min_cost)
    {
      min_cost = cost;
      best_idx = i;
    }
  }
  if (best_idx == -1)
  {
    ROS_WARN_THROTTLE(1.0, "[VFH] 局部死锁！");
    need_replan = true;
    return false;
  }
  float final_yaw = -M_PI + best_idx * (2 * M_PI / BINS) + (M_PI / BINS) * 0.5f + current_yaw;
  pub_viz_vfh_vectors(target_yaw, final_yaw, curr);
  float speed = std::min(cfg.max_speed, dist);
  if (std::abs(final_yaw - current_yaw) > 0.5)
    speed *= 0.5;
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
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = pt.x();
    pose.pose.position.y = pt.y();
    pose.pose.position.z = init_pos_z + cfg.takeoff_height;
    pose.pose.orientation.w = 1.0;
    msg.poses.push_back(pose);
  }
  pub_viz_path_raw.publish(msg);
}

void pub_viz_smooth_path(const std::vector<Eigen::Vector2f> &path)
{
  visualization_msgs::MarkerArray ma;
  visualization_msgs::Marker delete_msg;
  delete_msg.action = visualization_msgs::Marker::DELETEALL;
  delete_msg.header.frame_id = "map";
  ma.markers.push_back(delete_msg);
  for (size_t i = 0; i < path.size(); i += 2)
  {
    visualization_msgs::Marker mk;
    mk.header.frame_id = "map";
    mk.header.stamp = ros::Time::now();
    mk.ns = "smooth_traj";
    mk.id = i;
    mk.type = visualization_msgs::Marker::SPHERE;
    mk.action = visualization_msgs::Marker::ADD;
    mk.pose.position.x = path[i].x();
    mk.pose.position.y = path[i].y();
    mk.pose.position.z = init_pos_z + cfg.takeoff_height;
    mk.scale.x = 0.15;
    mk.scale.y = 0.15;
    mk.scale.z = 0.15;
    mk.color.r = 0.0;
    mk.color.g = 0.8;
    mk.color.b = 1.0;
    mk.color.a = 0.6;
    mk.pose.orientation.w = 1.0;
    ma.markers.push_back(mk);
  }
  pub_viz_path_smooth.publish(ma);
}

void pub_viz_vfh_vectors(float target_yaw, float selected_yaw, const Eigen::Vector2f &pos)
{
  visualization_msgs::Marker m;
  m.header.frame_id = "map";
  m.header.stamp = ros::Time::now();
  m.ns = "vfh_vec";
  m.id = 0;
  m.type = visualization_msgs::Marker::ARROW;
  m.action = visualization_msgs::Marker::ADD;
  m.pose.position.x = pos.x();
  m.pose.position.y = pos.y();
  m.pose.position.z = init_pos_z + cfg.takeoff_height;
  m.scale.x = 1.0;
  m.scale.y = 0.05;
  m.scale.z = 0.05;
  m.pose.orientation.w = 1.0;
  m.color.r = 1.0;
  m.color.g = 0.0;
  m.color.b = 0.0;
  m.color.a = 1.0;
  tf::quaternionTFToMsg(tf::createQuaternionFromYaw(target_yaw), m.pose.orientation);
  pub_viz_vfh.publish(m);
  m.id = 1;
  m.color.r = 0.0;
  m.color.g = 1.0;
  m.color.b = 0.0;
  m.color.a = 1.0;
  tf::quaternionTFToMsg(tf::createQuaternionFromYaw(selected_yaw), m.pose.orientation);
  pub_viz_vfh.publish(m);
}

void pub_viz_grid_map(const OccupancyGrid2D &grid)
{
  nav_msgs::OccupancyGrid msg;
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = "map";
  msg.info.resolution = grid.resolution;
  msg.info.width = OccupancyGrid2D::GRID_W;
  msg.info.height = OccupancyGrid2D::GRID_H;
  msg.info.origin.position.x = grid.origin_x;
  msg.info.origin.position.y = grid.origin_y;
  msg.info.origin.orientation.w = 1.0;
  msg.data.resize(msg.info.width * msg.info.height);
  for (int i = 0; i < msg.data.size(); ++i)
  {
    int x = i % msg.info.width;
    int y = i / msg.info.width;
    int val = grid.cells[x][y];
    msg.data[i] = (int8_t)((val > 100) ? 100 : val);
  }
  pub_viz_map.publish(msg);
}
// ------------------ 占位符 END ------------------

bool execute_avoidance_step(Eigen::Vector2f goal)
{
  Eigen::Vector2f curr(local_pos.pose.pose.position.x, local_pos.pose.pose.position.y);

  global_grid.update_with_memory(obstacles, cfg.uav_radius, cfg.safe_margin);
  static int cnt = 0;
  if (cnt++ % 5 == 0)
    pub_viz_grid_map(global_grid);

  bool blocked = is_path_blocked(global_path_smooth, global_grid, 0.0f);
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
      pub_viz_astar_path(global_path_raw);
      pub_viz_smooth_path(global_path_smooth);
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
    bool reached = run_vfh_plus(la, obstacles, stuck);
    if (stuck)
      has_global_plan = false;

    if ((curr - goal).norm() < 0.3)
      return true;
  }
  return false;
}

// 状态枚举
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
  LANDING,
  FINISHED
};

void state_cb(const mavros_msgs::State::ConstPtr &msg) { mavros_connection_state = *msg; }
void local_pos_cb(const nav_msgs::Odometry::ConstPtr &msg)
{
  local_pos = *msg;
  tf::quaternionMsgToTF(local_pos.pose.pose.orientation, quat);
  double r, p;
  tf::Matrix3x3(quat).getRPY(r, p, current_yaw);

  if (!flag_init_pos && local_pos.pose.pose.position.z > -0.5)
  {
    init_pos_x = local_pos.pose.pose.position.x;
    init_pos_y = local_pos.pose.pose.position.y;
    init_pos_z = local_pos.pose.pose.position.z;
    init_yaw_take_off = current_yaw;
    flag_init_pos = true;
  }
  // 兼容 PCL 的 flag
  flag_init_position = flag_init_pos;
}

int main(int argc, char **argv)
{
  setlocale(LC_ALL, "");
  ros::init(argc, argv, "astar_node");
  ros::NodeHandle nh("~");
  ros::NodeHandle public_nh;
  load_parameters(nh);

  // 1. 初始化 TF 监听器
  tf_listener = new tf::TransformListener();

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

  // 航点定义
  float wp1_x = 0.0, wp1_y = 4.0;
  float wp2_x = 3.5, wp2_y = 4.0;
  float wp3_x = 3.5, wp3_y = 0.0;
  float wp4_x = 6.0, wp4_y = 0.0;

  while (ros::ok())
  {
    pub_setpoint.publish(setpoint_raw);
    mission_step = (int)state;

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
      setpoint_raw.position.z = init_pos_z + cfg.takeoff_height;
      setpoint_raw.position.x = init_pos_x;
      setpoint_raw.position.y = init_pos_y;
      // 起飞时更新一次地图，建立初始记忆
      global_grid.update_with_memory(obstacles, cfg.uav_radius, cfg.safe_margin);
      if (std::abs(local_pos.pose.pose.position.z - setpoint_raw.position.z) < 0.2)
      {
        state = LEG1_AVOID;
        has_global_plan = false;
        ROS_INFO(">>> 避障前往 WP1");
      }
      break;

    case LEG1_AVOID:
      if (execute_avoidance_step({init_pos_x + wp1_x, init_pos_y + wp1_y}))
      {
        state = TURN1;
        ROS_INFO(">>> 转向右");
      }
      break;

    case TURN1:
      setpoint_raw.yaw = init_yaw_take_off - M_PI / 2.0;
      if (get_yaw_diff(setpoint_raw.yaw) < 0.1)
      {
        ros::Duration(1.0).sleep();
        state = LEG2_CROSS;
        ring_ctrl.reset();
        ROS_INFO(">>> 视觉穿门");
      }
      break;

    case LEG2_CROSS:
      if (ring_ctrl.compute_cmd(local_pos, current_yaw, setpoint_raw) || get_dist(wp2_x, wp2_y) < 0.5)
      {
        state = RECOVER;
        ROS_INFO(">>> 恢复位置控制");
        setpoint_raw.type_mask = 0b101111111000;
        setpoint_raw.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
        setpoint_raw.position.z = init_pos_z + cfg.takeoff_height;
      }
      break;

    case RECOVER:
      setpoint_raw.position.x = init_pos_x + wp2_x;
      setpoint_raw.position.y = init_pos_y + wp2_y;
      if (get_dist(wp2_x, wp2_y) < 0.3)
      {
        state = TURN2;
        ROS_INFO(">>> 转向下");
      }
      break;

    case TURN2:
      setpoint_raw.yaw = init_yaw_take_off + M_PI;
      if (get_yaw_diff(setpoint_raw.yaw) < 0.1)
      {
        state = LEG3_AVOID;
        has_global_plan = false;
        ROS_INFO(">>> 避障前往 WP3");
      }
      break;

    case LEG3_AVOID:
      if (execute_avoidance_step({init_pos_x + wp3_x, init_pos_y + wp3_y}))
      {
        state = TURN3;
        ROS_INFO(">>> 转向右");
      }
      break;

    case TURN3:
      setpoint_raw.yaw = init_yaw_take_off - M_PI / 2.0;
      if (get_yaw_diff(setpoint_raw.yaw) < 0.1)
      {
        state = LEG4_FINAL;
        has_global_plan = false;
        ROS_INFO(">>> 前往终点");
      }
      break;

    case LEG4_FINAL:
      if (execute_avoidance_step({init_pos_x + wp4_x, init_pos_y + wp4_y}))
      {
        state = LANDING;
        ROS_INFO(">>> 降落");
      }
      break;

    case LANDING:
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