/**
 * @file astar.cpp
 * @brief 修复版：增加坐标系转换，解决“障碍物看不见”的问题
 */
#include "astar.h"
#include <iostream>
#include <algorithm>
#include <cmath>
#include <clocale>

// ============================================================================
// 1. 全局变量定义
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

// ============================================================================
// 2. 配置参数
// ============================================================================
struct Config
{
  float target_z;
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
// 参数加载
// ============================================================================
void load_parameters(ros::NodeHandle &nh)
{
  nh.param<float>("target_x", target_x, 5.0f);
  nh.param<float>("target_y", target_y, 0.0f);
  nh.param<float>("debug_mode", if_debug, 1.0f);

  nh.param<float>("target_z", cfg.target_z, 1.0f);
  nh.param<float>("planner/takeoff_height", cfg.takeoff_height, 1.2f);
  nh.param<float>("planner/uav_radius", cfg.uav_radius, 0.3f);
  nh.param<float>("planner/safe_margin", cfg.safe_margin, 0.3f);
  nh.param<float>("planner/max_speed", cfg.max_speed, 0.8f);

  nh.param<float>("planner/lookahead_dist", cfg.lookahead_dist, 1.5f);
  nh.param<float>("planner/astar_weight", cfg.astar_weight, 1.5f);
  nh.param<float>("planner/replan_cooldown", cfg.replan_cooldown, 1.0f);
  nh.param<int>("planner/map_decay_rate", cfg.map_decay_rate, 2);

  cfg.check_radius = cfg.uav_radius + 0.1f;

  ROS_INFO("=== 参数加载完成 ===");
}

// ============================================================================
// PCL 障碍物回调 (核心修复：增加坐标变换)
// ============================================================================
void detection_cb_wrapper(const pcl_detection::ObjectDetectionResult::ConstPtr &msg)
{
  if (!flag_init_position)
    return;

  // 只有在避障阶段才更新，防止起飞阶段地面干扰
  // 修改：允许在 mission_step >= 1 (起飞后) 就开始感知，以便观察
  if (mission_step < 1)
    return;

  obstacles.clear();
  int obs_id = 0;

  // 获取无人机当前世界坐标和朝向
  float drone_wx = local_pos.pose.pose.position.x;
  float drone_wy = local_pos.pose.pose.position.y;
  float drone_yaw = current_yaw; // 由 local_pos_cb 更新

  // 预计算旋转矩阵
  float cos_yaw = cos(drone_yaw);
  float sin_yaw = sin(drone_yaw);

  for (const auto &obj : msg->objects)
  {
    if (!std::isfinite(obj.position.x) || !std::isfinite(obj.position.y))
      continue;

    // === 坐标变换核心 ===
    // 假设 pcl_detection 输出的是相对于雷达/机身的坐标 (Body Frame)
    // X_body = 前, Y_body = 左
    float rel_x = obj.position.x;
    float rel_y = obj.position.y;

    // 旋转 + 平移 -> 世界坐标
    float abs_x = drone_wx + rel_x * cos_yaw - rel_y * sin_yaw;
    float abs_y = drone_wy + rel_x * sin_yaw + rel_y * cos_yaw;

    // 调试日志：打印前几个障碍物的坐标变换结果
    if (if_debug > 0.5 && obs_id == 0)
    {
      ROS_INFO_THROTTLE(1.0, "[感知] 机身:(%.1f, %.1f) | 相对:(%.1f, %.1f) -> 绝对:(%.1f, %.1f)",
                        drone_wx, drone_wy, rel_x, rel_y, abs_x, abs_y);
    }

    // 距离过滤 (基于相对距离)
    float dist_rel = std::hypot(rel_x, rel_y);
    if (dist_rel > 10.0f)
      continue;
    if (dist_rel < 0.5f)
      continue; // 过滤自身

    // 1. 圆柱/圆
    if (obj.type == 1 || obj.type == 2)
    {
      if (obj.radius > 3.0f)
        continue; // 过滤错误的大平面

      Obstacle obs;
      obs.id = obs_id++;
      obs.type = CYLINDER;
      obs.position = Eigen::Vector2f(abs_x, abs_y); // 使用绝对坐标
      obs.radius = obj.radius;
      obs.length = 0;
      obs.angle = 0;
      obstacles.push_back(obs);
    }
    // 2. 墙体
    else if (obj.type == 0)
    {
      if (obj.plane_coeffs.size() < 4)
        continue;

      // 法向量也是相对坐标系下的，需要旋转到世界坐标系
      double nx_rel = obj.plane_coeffs[0];
      double ny_rel = obj.plane_coeffs[1];
      double norm = std::sqrt(nx_rel * nx_rel + ny_rel * ny_rel);
      if (norm < 1e-3)
        continue;
      nx_rel /= norm;
      ny_rel /= norm;

      // 旋转法向量
      double nx_world = nx_rel * cos_yaw - ny_rel * sin_yaw;
      double ny_world = nx_rel * sin_yaw + ny_rel * cos_yaw;

      // 切向量 (-ny, nx)
      double tx = -ny_world;
      double ty = nx_world;

      Obstacle obs;
      obs.id = obs_id++;
      obs.type = WALL;
      obs.position = Eigen::Vector2f(abs_x, abs_y); // 使用绝对坐标
      obs.radius = 0.05f;
      obs.length = obj.width;
      obs.angle = std::atan2(ty, tx);

      obstacles.push_back(obs);
    }
  }
}

void livox_cb_wrapper(const livox_ros_driver::CustomMsg::ConstPtr &msg) {}

// ============================================================================
// 辅助函数
// ============================================================================
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
// 地图核心
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

  // 1. 三区衰减
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
          decay = 20;
        else if (dist > 20.0f)
          decay = 2;
        cells[i][j] = std::max(0, cells[i][j] - decay);
      }
    }
  }

  float total_margin = drone_r + safe_margin;

  for (const auto &obs : obstacles)
  {
    // 这里的 obs.position 已经是绝对坐标了
    if ((obs.position - drone_p).norm() < 0.6f)
      continue;

    if (obs.type == CYLINDER)
    {
      int gx, gy;
      if (!world_to_grid(obs.position.x(), obs.position.y(), gx, gy))
        continue;

      float effect_r = obs.radius + total_margin;
      int r_cells = std::ceil(effect_r / resolution);
      float r_sq = (effect_r / resolution) * (effect_r / resolution);

      for (int dx = -r_cells; dx <= r_cells; ++dx)
      {
        for (int dy = -r_cells; dy <= r_cells; ++dy)
        {
          int nx = gx + dx;
          int ny = gy + dy;
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
      float half_len = obs.length / 2.0f;
      Eigen::Vector2f dir(cos(obs.angle), sin(obs.angle));
      Eigen::Vector2f p1 = obs.position - dir * half_len;
      Eigen::Vector2f p2 = obs.position + dir * half_len;

      float expansion = obs.radius + total_margin;
      float exp_sq = expansion * expansion;

      float min_x = std::min(p1.x(), p2.x()) - expansion;
      float max_x = std::max(p1.x(), p2.x()) + expansion;
      float min_y = std::min(p1.y(), p2.y()) - expansion;
      float max_y = std::max(p1.y(), p2.y()) + expansion;

      int min_gx, min_gy, max_gx, max_gy;
      if (!world_to_grid(min_x, min_y, min_gx, min_gy))
      {
        min_gx = 0;
        min_gy = 0;
      }
      if (!world_to_grid(max_x, max_y, max_gx, max_gy))
      {
        max_gx = GRID_W - 1;
        max_gy = GRID_H - 1;
      }

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
          Eigen::Vector2f grid_pt(wx, wy);
          if (dist_sq_point_to_segment(grid_pt, p1, p2) <= exp_sq)
          {
            cells[x][y] = MAX_HEALTH;
          }
        }
      }
    }
  }
}

// ============================================================================
// 可视化
// ============================================================================
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
    if (val > 100)
      val = 100;
    msg.data[i] = (int8_t)val;
  }
  pub_viz_map.publish(msg);
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
      float b0 = (1 - u) * (1 - u) * (1 - u) / 6.0f;
      float b1 = (3 * u * u * u - 6 * u * u + 4) / 6.0f;
      float b2 = (-3 * u * u * u + 3 * u * u + 3 * u + 1) / 6.0f;
      float b3 = u * u * u / 6.0f;
      result.push_back(b0 * pts[i] + b1 * pts[i + 1] + b2 * pts[i + 2] + b3 * pts[i + 3]);
    }
  }
  result.push_back(cps.back());
  return result;
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
    bool vis[OccupancyGrid2D::GRID_W][OccupancyGrid2D::GRID_H] = {false};
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
        if (nx >= 0 && nx < OccupancyGrid2D::GRID_W && ny >= 0 && ny < OccupancyGrid2D::GRID_H && !vis[nx][ny])
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
  int start_id = sgx * OccupancyGrid2D::GRID_H + sgy;
  int goal_id = ggx * OccupancyGrid2D::GRID_H + ggy;
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
    int cx = curr / OccupancyGrid2D::GRID_H;
    int cy = curr % OccupancyGrid2D::GRID_H;
    int dx[] = {1, -1, 0, 0, 1, 1, -1, -1};
    int dy[] = {0, 0, 1, -1, 1, -1, 1, -1};
    float dists[] = {1, 1, 1, 1, 1.414, 1.414, 1.414, 1.414};
    for (int i = 0; i < 8; ++i)
    {
      int nx = cx + dx[i], ny = cy + dy[i];
      if (nx < 0 || nx >= OccupancyGrid2D::GRID_W || ny < 0 || ny >= OccupancyGrid2D::GRID_H)
        continue;
      if (grid.is_occupied(nx, ny))
        continue;
      int n_id = nx * OccupancyGrid2D::GRID_H + ny;
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
    grid.grid_to_world(curr / OccupancyGrid2D::GRID_H, curr % OccupancyGrid2D::GRID_H, wx, wy);
    raw_pts.push_back(Eigen::Vector2f(wx, wy));
    curr = parent[curr];
  }
  std::reverse(raw_pts.begin(), raw_pts.end());
  if (!raw_pts.empty())
  {
    out_path.push_back(raw_pts[0]);
    for (size_t i = 1; i < raw_pts.size() - 1; ++i)
    {
      if ((raw_pts[i] - out_path.back()).norm() > 0.6)
      {
        out_path.push_back(raw_pts[i]);
      }
    }
    out_path.push_back(raw_pts.back());
  }
  return true;
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

// ============================================================================
// 主函数
// ============================================================================
void state_cb(const mavros_msgs::State::ConstPtr &msg) { mavros_connection_state = *msg; }
void local_pos_cb(const nav_msgs::Odometry::ConstPtr &msg)
{
  local_pos = *msg;
  tf::quaternionMsgToTF(local_pos.pose.pose.orientation, quat);
  double r, p;
  tf::Matrix3x3(quat).getRPY(r, p, current_yaw);

  if (!flag_init_position && local_pos.pose.pose.position.z > -0.5)
  {
    init_position_x_take_off = local_pos.pose.pose.position.x;
    init_position_y_take_off = local_pos.pose.pose.position.y;
    init_position_z_take_off = local_pos.pose.pose.position.z;
    init_yaw_take_off = current_yaw;
    flag_init_position = true;
    ROS_INFO("起飞点已初始化: (%.2f, %.2f, %.2f)", init_position_x_take_off, init_position_y_take_off, init_position_z_take_off);
  }
}

int main(int argc, char **argv)
{
  setlocale(LC_ALL, "");
  ros::init(argc, argv, "astar_node");
  ros::NodeHandle nh("~");
  ros::NodeHandle public_nh;
  load_parameters(nh);

  ros::Subscriber sub_state = public_nh.subscribe("mavros/state", 10, state_cb);
  ros::Subscriber sub_pos = public_nh.subscribe("/mavros/local_position/odom", 10, local_pos_cb);
  ros::Subscriber sub_detection = public_nh.subscribe("/pcl_detection/result", 10, detection_cb_wrapper);

  pub_setpoint = public_nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 10);
  pub_viz_path_raw = public_nh.advertise<nav_msgs::Path>("/viz/raw_path", 1);
  pub_viz_path_smooth = public_nh.advertise<visualization_msgs::MarkerArray>("/viz/smooth_path", 1);
  pub_viz_vfh = public_nh.advertise<visualization_msgs::Marker>("/viz/vfh_vec", 1);
  pub_viz_map = public_nh.advertise<nav_msgs::OccupancyGrid>("/viz/grid_map", 1, true);

  ros::ServiceClient client_arm = public_nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
  ros::ServiceClient client_mode = public_nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

  ros::Rate rate(20.0);
  ROS_INFO("等待飞控连接...");
  while (ros::ok() && (!mavros_connection_state.connected || local_pos.header.seq == 0))
  {
    ros::spinOnce();
    rate.sleep();
  }

  int input;
  std::cout << "请输入 1 开始任务: ";
  std::cin >> input;
  if (input != 1)
    return 0;

  setpoint_raw.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
  setpoint_raw.type_mask = 0b101111111000;
  setpoint_raw.position.x = local_pos.pose.pose.position.x;
  setpoint_raw.position.y = local_pos.pose.pose.position.y;
  setpoint_raw.position.z = local_pos.pose.pose.position.z;
  setpoint_raw.yaw = current_yaw;

  for (int i = 0; i < 50; ++i)
  {
    pub_setpoint.publish(setpoint_raw);
    ros::spinOnce();
    rate.sleep();
  }

  int mission_step = 0;
  bool has_global_plan = false;
  ros::Time last_req = ros::Time::now();
  OccupancyGrid2D global_grid;

  while (ros::ok())
  {
    pub_setpoint.publish(setpoint_raw);

    switch (mission_step)
    {
    case 0: // 解锁
      if (mavros_connection_state.mode != "OFFBOARD" && (ros::Time::now() - last_req > ros::Duration(5.0)))
      {
        mavros_msgs::SetMode srv;
        srv.request.custom_mode = "OFFBOARD";
        if (client_mode.call(srv))
          ROS_INFO("请求 OFFBOARD");
        last_req = ros::Time::now();
      }
      else if (mavros_connection_state.mode == "OFFBOARD" && !mavros_connection_state.armed && (ros::Time::now() - last_req > ros::Duration(5.0)))
      {
        mavros_msgs::CommandBool srv;
        srv.request.value = true;
        if (client_arm.call(srv))
          ROS_INFO("请求解锁");
        last_req = ros::Time::now();
      }
      if (mavros_connection_state.armed)
      {
        if (!flag_init_pos)
        {
          init_pos_x = local_pos.pose.pose.position.x;
          init_pos_y = local_pos.pose.pose.position.y;
          init_pos_z = local_pos.pose.pose.position.z;
          flag_init_pos = true;
        }
        mission_step = 1;
        ROS_INFO("起飞中...");
      }
      break;

    case 1: // 爬升
    {
      float target_abs_z = init_pos_z + cfg.takeoff_height;
      setpoint_raw.position.z = target_abs_z;
      setpoint_raw.position.x = init_pos_x;
      setpoint_raw.position.y = init_pos_y;

      // 起飞阶段开启地图更新，但不用于避障，只用于建立初始记忆
      global_grid.update_with_memory(obstacles, cfg.uav_radius, cfg.safe_margin);

      if (std::abs(local_pos.pose.pose.position.z - target_abs_z) < 0.2)
      {
        mission_step = 2;
        ROS_INFO("高度到达，开始规划逻辑");
      }
      break;
    }

    case 2: // 规划与避障
    {
      Eigen::Vector2f curr(local_pos.pose.pose.position.x, local_pos.pose.pose.position.y);
      Eigen::Vector2f goal(init_pos_x + target_x, init_pos_y + target_y);
      global_grid.update_with_memory(obstacles, cfg.uav_radius, cfg.safe_margin);

      static int cnt = 0;
      if (cnt++ % 5 == 0)
        pub_viz_grid_map(global_grid);

      bool blocked = is_path_blocked(global_path_smooth, global_grid, 0.0f);
      bool cooldown_over = (ros::Time::now() - last_replan_time).toSec() > cfg.replan_cooldown;

      if (!has_global_plan || (blocked && cooldown_over))
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
          ROS_WARN_THROTTLE(1.0, "A* 规划失败，重试中...");
          has_global_plan = false;
        }
      }

      if (has_global_plan)
      {
        Eigen::Vector2f lookahead_pt = get_lookahead_point(global_path_smooth, curr, cfg.lookahead_dist);
        bool vfh_stuck = false;
        bool reached = run_vfh_plus(lookahead_pt, obstacles, vfh_stuck);
        if (vfh_stuck)
          has_global_plan = false;
        if ((curr - goal).norm() < 0.3)
        {
          mission_step = 3;
          ROS_INFO("到达目标点，准备降落");
        }
      }
      break;
    }

    case 3: // 降落
      setpoint_raw.position.z = local_pos.pose.pose.position.z - 0.1;
      if (local_pos.pose.pose.position.z < init_pos_z + 0.1)
      {
        ROS_INFO("落地，任务结束");
        return 0;
      }
      break;
    }
    ros::spinOnce();
    rate.sleep();
  }
  return 0;
}