/**
 * @file astar.cpp
 * @brief 无人机避障规划系统 (中文调试 + 可视化修复版)
 */
#include "astar.h"
#include <iostream>
#include <algorithm>
#include <cmath>
#include <clocale> // 引入本地化头文件

// ============================================================================
// 1. 全局变量定义 (必须与 new_detect_obs.h 中的 extern 严格对应)
// ============================================================================
float target_x = 0.0f;
float target_y = 0.0f;
float if_debug = 1.0f;

float init_position_x_take_off = 0;
float init_position_y_take_off = 0;
float init_position_z_take_off = 0;
float init_yaw_take_off = 0;
bool flag_init_position = false;

// ============================================================================
// 其他全局变量
// ============================================================================
int mission_step = 0; // 0:初始, 1:起飞, 2:避障, 3:降落

mavros_msgs::PositionTarget setpoint_raw;
mavros_msgs::State mavros_connection_state;
nav_msgs::Odometry local_pos;
double current_yaw = 0.0;
tf::Quaternion quat;

// ============================================================================
// 规划器配置参数
// ============================================================================
struct PlannerParams
{
  float target_z;

  // 物理参数
  float uav_radius;
  float safe_margin;
  float max_speed;
  float min_safe_dist;

  // 规划参数
  float corridor_width;
  float replan_cooldown;
  bool enable_corridor;
} cfg;

// ============================================================================
// 内部变量
// ============================================================================
enum AvoidanceState
{
  PLANNING,
  AVOIDING,
  REPLANNING
};
AvoidanceState avoidance_state = PLANNING;

ros::Time last_replan_time;
const int MAX_PATH_POINTS = 500;
float path_x[MAX_PATH_POINTS] = {0};
float path_y[MAX_PATH_POINTS] = {0};
int path_len = 0;

const int MAX_CORR_POINTS = 500;
float corr_x[MAX_CORR_POINTS] = {0};
float corr_y[MAX_CORR_POINTS] = {0};
float corr_w[MAX_CORR_POINTS] = {0};
int corr_len = 0;

// ROS Publishers
ros::Publisher pub_setpoint;
ros::Publisher pub_viz_path; // A* 路径
ros::Publisher pub_viz_corr; // 走廊
ros::Publisher pub_viz_vfh;  // VFH 向量

// ============================================================================
// 参数加载
// ============================================================================
void load_parameters(ros::NodeHandle &nh)
{
  nh.param<float>("target_x", target_x, 5.0f);
  nh.param<float>("target_y", target_y, 0.0f);
  nh.param<float>("debug_mode", if_debug, 1.0f);

  nh.param<float>("target_z", cfg.target_z, 1.0f);

  nh.param<float>("planner/uav_radius", cfg.uav_radius, 0.3f);
  nh.param<float>("planner/safe_margin", cfg.safe_margin, 0.3f);
  nh.param<float>("planner/max_speed", cfg.max_speed, 0.8f);
  nh.param<float>("planner/min_safe_dist", cfg.min_safe_dist, 0.3f);

  nh.param<float>("planner/corridor_width", cfg.corridor_width, 1.0f);
  nh.param<float>("planner/replan_cooldown", cfg.replan_cooldown, 2.0f);
  nh.param<bool>("planner/enable_corridor", cfg.enable_corridor, true);

  ROS_INFO("=== 参数加载完成 ===");
  ROS_INFO("目标相对坐标: (%.1f, %.1f), 调试模式: %.0f", target_x, target_y, if_debug);
}

// ============================================================================
// 可视化函数实现 (确保 Frame ID 正确)
// ============================================================================
void pub_viz_astar_path(const float *x, const float *y, int num)
{
  nav_msgs::Path path_msg;
  path_msg.header.stamp = ros::Time::now();
  // 重要：确保 RViz 中的 Fixed Frame 设置为 map
  path_msg.header.frame_id = "map";

  for (int i = 0; i < num; ++i)
  {
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = x[i];
    pose.pose.position.y = y[i];
    pose.pose.position.z = init_position_z_take_off + cfg.target_z;
    pose.pose.orientation.w = 1.0;
    path_msg.poses.push_back(pose);
  }
  pub_viz_path.publish(path_msg);
}

void pub_viz_corridor(const float *x, const float *y, const float *w, int num)
{
  visualization_msgs::MarkerArray ma;
  // 先发送清除指令，防止残留
  visualization_msgs::Marker delete_all;
  delete_all.action = visualization_msgs::Marker::DELETEALL;
  delete_all.header.frame_id = "map";
  ma.markers.push_back(delete_all);

  for (int i = 0; i < num; i += 2)
  {
    visualization_msgs::Marker mk;
    mk.header.frame_id = "map";
    mk.header.stamp = ros::Time::now();
    mk.ns = "corridor";
    mk.id = i;
    mk.type = visualization_msgs::Marker::CYLINDER;
    mk.action = visualization_msgs::Marker::ADD;
    mk.pose.position.x = x[i];
    mk.pose.position.y = y[i];
    mk.pose.position.z = init_position_z_take_off + cfg.target_z;
    mk.scale.x = w[i]; // 圆柱直径 = 走廊宽
    mk.scale.y = w[i];
    mk.scale.z = 0.05; // 薄片
    mk.color.r = 1.0;
    mk.color.g = 1.0;
    mk.color.b = 0.0;
    mk.color.a = 0.3; // 黄色半透明
    ma.markers.push_back(mk);
  }
  pub_viz_corr.publish(ma);
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
  m.pose.position.z = init_position_z_take_off + cfg.target_z;

  // 1. 目标方向 (红色箭头)
  m.scale.x = 1.0;
  m.scale.y = 0.05;
  m.scale.z = 0.05;
  m.color.r = 1.0;
  m.color.g = 0.0;
  m.color.b = 0.0;
  m.color.a = 1.0;
  tf::quaternionTFToMsg(tf::createQuaternionFromYaw(target_yaw), m.pose.orientation);
  pub_viz_vfh.publish(m);

  // 2. VFH选择方向 (绿色箭头)
  m.id = 1;
  m.color.r = 0.0;
  m.color.g = 1.0;
  m.color.b = 0.0;
  m.color.a = 1.0;
  tf::quaternionTFToMsg(tf::createQuaternionFromYaw(selected_yaw), m.pose.orientation);
  pub_viz_vfh.publish(m);
}

// ============================================================================
// 地图与A*实现
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
  return cells[gx][gy] > 50;
}

void OccupancyGrid2D::update_with_obstacles(const std::vector<Obstacle> &obstacles, float drone_r, float safe_margin)
{
  for (int i = 0; i < GRID_W; ++i)
    for (int j = 0; j < GRID_H; ++j)
      cells[i][j] = 0;

  float total_r = drone_r + safe_margin;
  for (const auto &obs : obstacles)
  {
    float obs_x = obs.position.x();
    float obs_y = obs.position.y();

    int gx, gy;
    if (!world_to_grid(obs_x, obs_y, gx, gy))
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
            cells[nx][ny] = 100;
        }
      }
    }
  }
}

int astar_plan(const OccupancyGrid2D &grid, float start_x, float start_y, float goal_x, float goal_y, float *path_x, float *path_y, int max_points)
{
  int sgx, sgy, ggx, ggy;
  if (!grid.world_to_grid(start_x, start_y, sgx, sgy) || !grid.world_to_grid(goal_x, goal_y, ggx, ggy))
  {
    ROS_ERROR("A* 错误：起点或终点在地图外！");
    return 0;
  }

  auto find_free = [&](int &cx, int &cy) -> bool
  {
    if (!grid.is_occupied(cx, cy))
      return true;
    std::queue<std::pair<int, int>> q;
    q.push({cx, cy});
    bool vis[OccupancyGrid2D::GRID_W][OccupancyGrid2D::GRID_H] = {false};
    vis[cx][cy] = true;
    int steps = 200;
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
  {
    ROS_WARN("起点被占用，搜索最近点...");
    find_free(sgx, sgy);
  }
  if (grid.is_occupied(ggx, ggy))
  {
    ROS_WARN("终点被占用，搜索最近点...");
    find_free(ggx, ggy);
  }

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
  while (!open.empty() && iter++ < 10000)
  {
    auto top = open.top();
    open.pop();
    int curr = top.second;
    if (curr == goal_id)
    {
      found = true;
      break;
    }
    if (top.first > g_cost[curr] + 500)
      continue;

    int cx = curr / OccupancyGrid2D::GRID_H;
    int cy = curr % OccupancyGrid2D::GRID_H;

    int dx[] = {1, -1, 0, 0}, dy[] = {0, 0, 1, -1};
    for (int i = 0; i < 4; ++i)
    {
      int nx = cx + dx[i], ny = cy + dy[i];

      if (nx < 0 || nx >= OccupancyGrid2D::GRID_W || ny < 0 || ny >= OccupancyGrid2D::GRID_H)
        continue;
      if (grid.cells[nx][ny] > 50)
        continue;

      int n_id = nx * OccupancyGrid2D::GRID_H + ny;
      float new_g = g_cost[curr] + 1.0;

      if (new_g < g_cost[n_id])
      {
        g_cost[n_id] = new_g;
        parent[n_id] = curr;
        float h = std::hypot(nx - ggx, ny - ggy);
        open.push({new_g + h * 1.05, n_id});
      }
    }
  }

  if (!found)
  {
    ROS_ERROR("A* 规划失败！无法找到路径");
    return 0;
  }

  std::vector<std::pair<int, int>> path;
  int curr = goal_id;
  while (curr != -1)
  {
    path.push_back({curr / OccupancyGrid2D::GRID_H, curr % OccupancyGrid2D::GRID_H});
    curr = parent[curr];
  }
  std::reverse(path.begin(), path.end());

  int cnt = 0;
  for (const auto &pt : path)
  {
    if (cnt >= max_points)
      break;
    float wx, wy;
    grid.grid_to_world(pt.first, pt.second, wx, wy);
    path_x[cnt] = wx;
    path_y[cnt] = wy;
    cnt++;
  }
  return cnt;
}

int generate_corridor(const float *ax, const float *ay, int num, float base_w, float *cx, float *cy, float *cw, int max_size)
{
  int cnt = 0;
  for (int i = 0; i < num; ++i)
  {
    if (cnt >= max_size)
      break;
    cx[cnt] = ax[i];
    cy[cnt] = ay[i];
    cw[cnt] = base_w;
    cnt++;
  }
  return cnt;
}

// ============================================================================
// 核心机制：路径有效性检测
// ============================================================================
bool check_global_path_blocked(const float *path_x, const float *path_y, int path_size, const std::vector<Obstacle> &obs, float check_radius)
{
  for (int i = 0; i < path_size; i += 3)
  {
    float px = path_x[i];
    float py = path_y[i];

    float dist_to_drone = std::hypot(px - local_pos.pose.pose.position.x, py - local_pos.pose.pose.position.y);
    if (dist_to_drone > 6.0f)
      continue;
    if (dist_to_drone < 0.5f)
      continue;

    for (const auto &o : obs)
    {
      float dist_pt_obs = std::hypot(px - o.position.x(), py - o.position.y());
      if (dist_pt_obs < o.radius + check_radius)
      {
        if (if_debug > 0.5)
          ROS_WARN("[检测] 全局路径被障碍物截断 (Dist: %.2f)", dist_pt_obs);
        return true;
      }
    }
  }
  return false;
}

// ============================================================================
// VFH+ 实现
// ============================================================================
bool vfh_plus_with_corridor(float tx_rel, float ty_rel, float tyaw,
                            const float *cx, const float *cy, const float *cw, int c_size,
                            bool &out_replan)
{
  out_replan = false;
  float drone_x = local_pos.pose.pose.position.x;
  float drone_y = local_pos.pose.pose.position.y;

  float target_wx = init_position_x_take_off + tx_rel;
  float target_wy = init_position_y_take_off + ty_rel;

  float dx = target_wx - drone_x;
  float dy = target_wy - drone_y;
  float dist = std::hypot(dx, dy);

  if (dist < 0.3)
  {
    setpoint_raw.position.x = drone_x;
    setpoint_raw.position.y = drone_y;
    return true;
  }

  const int H_SIZE = 72;
  float hist[H_SIZE] = {0};

  for (const auto &o : obstacles)
  {
    float ox = o.position.x() - drone_x;
    float oy = o.position.y() - drone_y;
    float d = std::hypot(ox, oy);

    if (d > 4.5 || d < 0.1)
      continue;

    float angle = std::atan2(oy, ox) - current_yaw;
    while (angle > M_PI)
      angle -= 2 * M_PI;
    while (angle < -M_PI)
      angle += 2 * M_PI;

    float effect_r = o.radius + cfg.uav_radius + cfg.safe_margin;
    float val = effect_r / d;
    if (val > 1.0f)
      val = 1.0f;
    float width_ang = std::asin(val);

    int center_idx = (int)((angle + M_PI) / (2 * M_PI) * H_SIZE) % H_SIZE;
    int half_width = (int)(width_ang / (2 * M_PI) * H_SIZE) + 1;

    for (int k = center_idx - half_width; k <= center_idx + half_width; ++k)
    {
      int idx = (k + H_SIZE) % H_SIZE;
      hist[idx] += 10.0f / d;
    }
  }

  float goal_ang = std::atan2(dy, dx) - current_yaw;
  while (goal_ang > M_PI)
    goal_ang -= 2 * M_PI;
  while (goal_ang < -M_PI)
    goal_ang += 2 * M_PI;

  int best_idx = -1;
  float min_cost = 1e9;

  for (int i = 0; i < H_SIZE; ++i)
  {
    if (hist[i] > 15.0)
      continue;

    float bin_ang = -M_PI + i * (2 * M_PI / H_SIZE) + (M_PI / H_SIZE) * 0.5;
    float cost = std::abs(bin_ang - goal_ang) + hist[i] * 0.2f;

    if (cost < min_cost)
    {
      min_cost = cost;
      best_idx = i;
    }
  }

  if (best_idx == -1)
  {
    ROS_WARN_THROTTLE(1.0, "[VFH] 局部死锁！所有方向被阻挡");
    out_replan = true;
    return false;
  }

  float final_yaw = -M_PI + best_idx * (2 * M_PI / H_SIZE) + current_yaw;

  if (cfg.enable_corridor && c_size > 0)
  {
    int near_idx = -1;
    float min_d = 1e9;
    for (int i = 0; i < c_size; ++i)
    {
      float d = std::hypot(cx[i] - drone_x, cy[i] - drone_y);
      if (d < min_d)
      {
        min_d = d;
        near_idx = i;
      }
    }

    if (near_idx != -1)
    {
      float hw = cw[near_idx] / 2.0;
      if (min_d > hw)
      {
        float corr_ang = std::atan2(cy[near_idx] - drone_y, cx[near_idx] - drone_x);
        final_yaw = final_yaw * 0.4 + corr_ang * 0.6;
      }
    }
  }

  pub_viz_vfh_vectors(goal_ang + current_yaw, final_yaw, Eigen::Vector2f(drone_x, drone_y));

  float speed = std::min(cfg.max_speed, dist);
  if (std::abs(final_yaw - current_yaw) > 0.5)
    speed *= 0.5;

  setpoint_raw.position.x = drone_x + std::cos(final_yaw) * speed * 0.5;
  setpoint_raw.position.y = drone_y + std::sin(final_yaw) * speed * 0.5;
  setpoint_raw.position.z = init_position_z_take_off + cfg.target_z;
  setpoint_raw.yaw = final_yaw;

  return false;
}

// ============================================================================
// 主循环
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
  // 1. 设置中文环境
  setlocale(LC_ALL, "");

  ros::init(argc, argv, "astar_node"); // 节点名最好和 launch 文件对应
  ros::NodeHandle nh;

  load_parameters(nh);

  ros::Subscriber sub_state = nh.subscribe("mavros/state", 10, state_cb);
  ros::Subscriber sub_pos = nh.subscribe("/mavros/local_position/odom", 10, local_pos_cb);
  ros::Subscriber sub_livox = nh.subscribe("/livox/lidar", 10, livox_cb_wrapper);

  pub_setpoint = nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 10);

  // 可视化 Topic
  pub_viz_path = nh.advertise<nav_msgs::Path>("/viz/astar_path", 1);
  pub_viz_corr = nh.advertise<visualization_msgs::MarkerArray>("/viz/corridor", 1);
  pub_viz_vfh = nh.advertise<visualization_msgs::Marker>("/viz/vfh_vec", 1);

  ros::ServiceClient client_arm = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
  ros::ServiceClient client_mode = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

  ros::Rate rate(20.0);

  ROS_INFO("等待飞控连接和 GPS 初始化...");
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

  // 预发送
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

  mission_step = 0;
  ros::Time last_req = ros::Time::now();

  while (ros::ok())
  {
    pub_setpoint.publish(setpoint_raw);

    switch (mission_step)
    {
    case 0: // 解锁起飞
      if (mavros_connection_state.mode != "OFFBOARD" && (ros::Time::now() - last_req > ros::Duration(5.0)))
      {
        mavros_msgs::SetMode srv;
        srv.request.custom_mode = "OFFBOARD";
        if (client_mode.call(srv))
          ROS_INFO("已请求 OFFBOARD 模式");
        last_req = ros::Time::now();
      }
      else if (mavros_connection_state.mode == "OFFBOARD" && !mavros_connection_state.armed && (ros::Time::now() - last_req > ros::Duration(5.0)))
      {
        mavros_msgs::CommandBool srv;
        srv.request.value = true;
        if (client_arm.call(srv))
          ROS_INFO("已请求解锁");
        last_req = ros::Time::now();
      }

      if (mavros_connection_state.armed)
      {
        if (!flag_init_position)
        {
          init_position_x_take_off = local_pos.pose.pose.position.x;
          init_position_y_take_off = local_pos.pose.pose.position.y;
          init_position_z_take_off = local_pos.pose.pose.position.z;
          flag_init_position = true;
        }
        mission_step = 1;
        ROS_INFO("解锁成功，开始起飞...");
      }
      break;

    case 1: // 爬升
    {
      float target_abs_z = init_position_z_take_off + cfg.target_z;
      setpoint_raw.position.z = target_abs_z;
      setpoint_raw.position.x = init_position_x_take_off;
      setpoint_raw.position.y = init_position_y_take_off;

      if (std::abs(local_pos.pose.pose.position.z - target_abs_z) < 0.2)
      {
        mission_step = 2;
        avoidance_state = PLANNING;
        ROS_INFO("到达目标高度，开始 A* 规划...");
      }
      break;
    }

    case 2: // 避障主循环
    {
      switch (avoidance_state)
      {
      case PLANNING:
      {
        OccupancyGrid2D grid;
        grid.update_with_obstacles(obstacles, cfg.uav_radius, cfg.safe_margin);

        float goal_wx = init_position_x_take_off + target_x;
        float goal_wy = init_position_y_take_off + target_y;

        path_len = astar_plan(grid,
                              local_pos.pose.pose.position.x, local_pos.pose.pose.position.y,
                              goal_wx, goal_wy,
                              path_x, path_y, MAX_PATH_POINTS);

        if (path_len > 0)
        {
          corr_len = generate_corridor(path_x, path_y, path_len, cfg.corridor_width, corr_x, corr_y, corr_w, MAX_CORR_POINTS);

          pub_viz_astar_path(path_x, path_y, path_len);
          pub_viz_corridor(corr_x, corr_y, corr_w, corr_len);

          avoidance_state = AVOIDING;
          ROS_INFO("A* 规划成功，开始避障飞行 (路径点: %d)", path_len);
        }
        else
        {
          ROS_WARN_THROTTLE(2.0, "A* 规划失败，重试中...");
        }
        break;
      }

      case AVOIDING:
      {
        bool path_blocked = check_global_path_blocked(path_x, path_y, path_len, obstacles, cfg.uav_radius + 0.1);

        bool vfh_stuck = false;
        if (path_blocked)
        {
          ROS_WARN("检测到路径上有新障碍物！触发 A* 重规划");
          avoidance_state = REPLANNING;
          last_replan_time = ros::Time::now();
        }
        else
        {
          // 修正后的调用：去掉 enable_corridor 参数
          bool reached = vfh_plus_with_corridor(target_x, target_y, 0, corr_x, corr_y, corr_w, corr_len, vfh_stuck);

          if (vfh_stuck)
          {
            avoidance_state = REPLANNING;
            last_replan_time = ros::Time::now();
          }
          if (reached)
            mission_step = 3;
        }
        break;
      }

      case REPLANNING:
      {
        if (ros::Time::now() - last_replan_time > ros::Duration(cfg.replan_cooldown))
        {
          avoidance_state = PLANNING;
        }
        else
        {
          setpoint_raw.position.x = local_pos.pose.pose.position.x;
          setpoint_raw.position.y = local_pos.pose.pose.position.y;
        }
        break;
      }
      }
      break;
    }

    case 3:
      setpoint_raw.position.z = local_pos.pose.pose.position.z - 0.1;
      if (local_pos.pose.pose.position.z < init_position_z_take_off + 0.1)
      {
        ROS_INFO("已降落，任务完成。");
        return 0;
      }
      break;
    }

    ros::spinOnce();
    rate.sleep();
  }
  return 0;
}