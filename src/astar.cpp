/**
 * @file astar.cpp
 * @brief 修复版：YAML读取修复 + 地图记忆衰减机制
 */
#include "astar.h"
#include <iostream>
#include <algorithm>
#include <cmath>
#include <clocale>

// ============================================================================
// 全局变量 (与 new_detect_obs.h 兼容)
// ============================================================================
float target_x = 0.0f;
float target_y = 0.0f;
float if_debug = 1.0f;

float init_position_x_take_off = 0;
float init_position_y_take_off = 0;
float init_position_z_take_off = 0;
float init_yaw_take_off = 0;
bool flag_init_position = false;

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
// 配置参数
// ============================================================================
struct Config
{
  float target_z;
  float uav_radius;
  float safe_margin;
  float max_speed;
  float min_safe_dist;
  float lookahead_dist;
  float astar_weight;
  float replan_cooldown;
  float check_radius;

  // 新增：地图衰减参数
  int map_decay_rate; // 每次更新减少多少数值
} cfg;

ros::Publisher pub_setpoint;
ros::Publisher pub_viz_path_raw;
ros::Publisher pub_viz_path_smooth;
ros::Publisher pub_viz_vfh;
ros::Publisher pub_viz_map; // 地图发布

// ============================================================================
// 参数加载 (修复：使用私有句柄)
// ============================================================================
void load_parameters(ros::NodeHandle &nh)
{
  // 这里的 nh 必须是 ros::NodeHandle("~")

  // 1. 加载到全局变量 (兼容感知模块)
  nh.param<float>("target_x", target_x, 5.0f);
  nh.param<float>("target_y", target_y, 0.0f);
  nh.param<float>("debug_mode", if_debug, 1.0f);

  // 2. 加载内部配置
  nh.param<float>("target_z", cfg.target_z, 1.0f);
  nh.param<float>("planner/uav_radius", cfg.uav_radius, 0.3f);
  nh.param<float>("planner/safe_margin", cfg.safe_margin, 0.3f);
  nh.param<float>("planner/max_speed", cfg.max_speed, 0.8f);

  nh.param<float>("planner/lookahead_dist", cfg.lookahead_dist, 1.5f);
  nh.param<float>("planner/astar_weight", cfg.astar_weight, 1.5f);
  nh.param<float>("planner/replan_cooldown", cfg.replan_cooldown, 1.0f);

  nh.param<int>("planner/map_decay_rate", cfg.map_decay_rate, 5); // 默认每帧衰减5 (共100，约20帧消失)

  cfg.check_radius = cfg.uav_radius + 0.1f;

  ROS_INFO("=== 参数加载成功 (Private Namespace) ===");
  ROS_INFO("Target Rel: (%.2f, %.2f)", target_x, target_y);
  ROS_INFO("Map Decay Rate: %d", cfg.map_decay_rate);
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
    pose.pose.position.z = init_pos_z + cfg.target_z;
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
    mk.pose.position.z = init_pos_z + cfg.target_z;
    mk.scale.x = 0.1;
    mk.scale.y = 0.1;
    mk.scale.z = 0.1;
    mk.color.r = 0.0;
    mk.color.g = 1.0;
    mk.color.b = 1.0;
    mk.color.a = 0.8;
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
  m.pose.position.z = init_pos_z + cfg.target_z;
  m.scale.x = 1.0;
  m.scale.y = 0.05;
  m.scale.z = 0.05;

  // 目标(红)
  m.color.r = 1.0;
  m.color.g = 0.0;
  m.color.b = 0.0;
  m.color.a = 1.0;
  tf::quaternionTFToMsg(tf::createQuaternionFromYaw(target_yaw), m.pose.orientation);
  pub_viz_vfh.publish(m);

  // 选择(绿)
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
    // 显示阈值：大于 50 认为是障碍，显示黑色
    msg.data[i] = (grid.cells[x][y] > 50) ? 100 : 0;
  }
  pub_viz_map.publish(msg);
}

// ============================================================================
// B-Spline
// ============================================================================
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

// ============================================================================
// 地图核心：动态衰减逻辑
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
  // 只有当占用值 > 50 时才视为障碍，这提供了缓冲
  return cells[gx][gy] > 50;
}

void OccupancyGrid2D::update_with_decay(const std::vector<Obstacle> &obstacles, float drone_r, float safe_margin)
{
  // 1. 全局衰减 (记忆功能的核心)
  // 不再直接清零，而是慢慢减小数值
  for (int i = 0; i < GRID_W; ++i)
  {
    for (int j = 0; j < GRID_H; ++j)
    {
      if (cells[i][j] > 0)
      {
        cells[i][j] = std::max(0, cells[i][j] - cfg.map_decay_rate);
      }
    }
  }

  // 2. 观测更新 (观测到的地方设为 100)
  float total_r = drone_r + safe_margin;
  for (const auto &obs : obstacles)
  {
    int gx, gy;
    if (!world_to_grid(obs.position.x(), obs.position.y(), gx, gy))
      continue;

    float effect_r = obs.radius + total_r;
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
          {
            cells[nx][ny] = 100; // 刷新障碍物存在感
          }
        }
      }
    }
  }
}

// ============================================================================
// A* 规划
// ============================================================================
bool run_astar(const OccupancyGrid2D &grid, Eigen::Vector2f start, Eigen::Vector2f goal, std::vector<Eigen::Vector2f> &out_path)
{
  out_path.clear();
  int sgx, sgy, ggx, ggy;
  if (!grid.world_to_grid(start.x(), start.y(), sgx, sgy) || !grid.world_to_grid(goal.x(), goal.y(), ggx, ggy))
  {
    ROS_ERROR("A* 起点或终点在地图外！");
    return false;
  }

  // 严密性：如果起点被老障碍物的余影挡住，尝试搜索附近
  auto find_free = [&](int &cx, int &cy) -> bool
  {
    if (!grid.is_occupied(cx, cy))
      return true;
    std::queue<std::pair<int, int>> q;
    q.push({cx, cy});
    bool vis[OccupancyGrid2D::GRID_W][OccupancyGrid2D::GRID_H] = {false};
    vis[cx][cy] = true;
    int steps = 300;
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
  while (!open.empty() && iter++ < 15000)
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
      { // 抽稀
        out_path.push_back(raw_pts[i]);
      }
    }
    out_path.push_back(raw_pts.back());
  }

  return true;
}

// ============================================================================
// 增量检测 (使用双重阈值防止震荡)
// ============================================================================
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

    // 检查地图上的占用情况
    int gx, gy;
    if (grid.world_to_grid(path[i].x(), path[i].y(), gx, gy))
    {
      // 如果已经在障碍物里 (值 > 50)，肯定堵了
      // 这里我们用更严格的判定：只要 grid 值 > 80 (确信障碍) 就认为堵了
      // 衰减中的障碍 (50-80) 允许穿过，这就是“迟滞”
      if (grid.cells[gx][gy] > 80)
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
  setpoint_raw.position.z = init_pos_z + cfg.target_z;
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
}

int main(int argc, char **argv)
{
  setlocale(LC_ALL, "");
  ros::init(argc, argv, "astar_node");

  // 关键修正：使用私有句柄加载参数
  ros::NodeHandle nh("~");
  ros::NodeHandle public_nh; // 用于订阅发布

  load_parameters(nh);

  ros::Subscriber sub_state = public_nh.subscribe("mavros/state", 10, state_cb);
  ros::Subscriber sub_pos = public_nh.subscribe("/mavros/local_position/odom", 10, local_pos_cb);
  ros::Subscriber sub_livox = public_nh.subscribe("/livox/lidar", 10, livox_cb_wrapper);

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

  // 全局地图实例
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
      float target_abs_z = init_pos_z + cfg.target_z;
      setpoint_raw.position.z = target_abs_z;
      setpoint_raw.position.x = init_pos_x;
      setpoint_raw.position.y = init_pos_y;

      if (std::abs(local_pos.pose.pose.position.z - target_abs_z) < 0.2)
      {
        mission_step = 2;
        ROS_INFO("高度到达，开始规划逻辑");
      }
      break;
    }

    case 2: // 规划与避障核心循环
    {
      Eigen::Vector2f curr(local_pos.pose.pose.position.x, local_pos.pose.pose.position.y);
      Eigen::Vector2f goal(init_pos_x + target_x, init_pos_y + target_y);

      // 1. 更新全局地图 (带衰减)
      global_grid.update_with_decay(obstacles, cfg.uav_radius, cfg.safe_margin);

      // 发布地图调试
      static int cnt = 0;
      if (cnt++ % 5 == 0)
        pub_viz_grid_map(global_grid);

      // 2. 增量检查
      bool blocked = is_path_blocked(global_path_smooth, global_grid, 0.0f);
      bool cooldown_over = (ros::Time::now() - last_replan_time).toSec() > cfg.replan_cooldown;

      // 3. 重规划决策
      if (!has_global_plan || (blocked && cooldown_over))
      {
        if (blocked)
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

      // 4. 执行控制
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