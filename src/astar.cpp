#include "astar.h"
#include <iostream>
#include <algorithm>
#include <cmath>

// ============================================================================
// 全局变量定义
// ============================================================================
int mission_num = 0;
float if_debug = 0;
float err_max = 0.2;
const float HOVER_DURATION = 10.0f;
float hold_flag = false;

// 【可配置参数】
float target_x = 5.0f;
float target_y = 0.0f;
float target_z = ALTITUDE;
float target_yaw = 0.0f;
float UAV_radius = 0.3f;
float time_final = 70.0f;

// 避障参数
float safe_margin = 0.4f;
float MAX_SPEED = 0.9f;
float MIN_SAFE_DISTANCE = 0.25f;
float CORRIDOR_WIDTH_BASE = 0.8f;
bool ENABLE_CORRIDOR = true;
float REPLAN_COOLDOWN = 2.0f; // 缩短重规划冷却，反应更快

// 全局变量定义（匹配头文件 extern）
mavros_msgs::PositionTarget setpoint_raw;
Eigen::Vector2f current_pos;
Eigen::Vector2f current_vel;
mavros_msgs::State mavros_connection_state;
tf::Quaternion quat;
nav_msgs::Odometry local_pos;
double roll, pitch, yaw;
float init_position_x_take_off = 0;
float init_position_y_take_off = 0;
float init_position_z_take_off = 0;
float init_yaw_take_off = 0;
bool flag_init_position = false;

// 巡航任务变量
float mission_pos_cruise_last_position_x = 0;
float mission_pos_cruise_last_position_y = 0;
float mission_cruise_timeout = 180.0f;
ros::Time mission_cruise_start_time;
bool mission_cruise_timeout_flag = false;
bool mission_pos_cruise_flag = false;

// 降落任务变量
float precision_land_init_position_x = 0;
float precision_land_init_position_y = 0;
bool precision_land_init_position_flag = false;
bool hovor_done = false;
bool land_done = false;
ros::Time precision_land_last_time;

// 任务状态机
enum AvoidanceState
{
  PLANNING,
  AVOIDING,
  REPLANNING,
  TARGET_REACHED
};

AvoidanceState avoidance_state = PLANNING;
ros::Time state_start_time;
ros::Time last_replan_time;

// 路径与走廊数据
const int MAX_PATH_POINTS = 200;
float astar_path_x[MAX_PATH_POINTS] = {0};
float astar_path_y[MAX_PATH_POINTS] = {0};
int path_size = 0;

const int MAX_CORRIDOR_POINTS = 200;
float corridor_x[MAX_CORRIDOR_POINTS] = {0};
float corridor_y[MAX_CORRIDOR_POINTS] = {0};
float corridor_width[MAX_CORRIDOR_POINTS] = {0};
int corridor_size = 0;

ros::Publisher mavros_setpoint_pos_pub;

void print_param()
{
  std::cout << "=== 避障系统参数 ===" << std::endl;
  std::cout << "UAV_radius: " << UAV_radius << " m" << std::endl;
  std::cout << "safe_margin: " << safe_margin << " m" << std::endl;
  std::cout << "MAX_SPEED: " << MAX_SPEED << " m/s" << std::endl;
}

// ============================================================================
// OccupancyGrid2D 实现 (优化边界与初始化)
// ============================================================================
OccupancyGrid2D::OccupancyGrid2D()
{
  resolution = 0.1f;
  origin_x = -10.0f; // 扩大原点范围
  origin_y = -10.0f;
  for (int i = 0; i < 200; ++i)
    for (int j = 0; j < 200; ++j)
      cells[i][j] = 0;
}

bool OccupancyGrid2D::world_to_grid(float wx, float wy, int &gx, int &gy) const
{
  gx = static_cast<int>((wx - origin_x) / resolution);
  gy = static_cast<int>((wy - origin_y) / resolution);
  return (gx >= 0 && gx < 200 && gy >= 0 && gy < 200);
}

void OccupancyGrid2D::grid_to_world(int gx, int gy, float &wx, float &wy) const
{
  wx = origin_x + (gx + 0.5f) * resolution; // 加上 0.5 取中心
  wy = origin_y + (gy + 0.5f) * resolution;
}

bool OccupancyGrid2D::is_occupied(int gx, int gy) const
{
  if (gx < 0 || gx >= 200 || gy < 0 || gy >= 200)
    return true; // 边界即障碍
  return cells[gx][gy] > 50;
}

void OccupancyGrid2D::update_with_obstacles(const std::vector<Obstacle> &obstacles, float drone_radius, float safety_margin)
{
  // 1. 衰减旧障碍物 (模拟动态环境)
  for (int i = 0; i < 200; ++i)
    for (int j = 0; j < 200; ++j)
      if (cells[i][j] > 0)
        cells[i][j] = 0; // 工程上直接清空比衰减更安全，防止拖影

  float total_margin = drone_radius + safety_margin;
  int expansion_cells = static_cast<int>(std::ceil(total_margin / resolution));

  for (const auto &obs : obstacles)
  {
    int gx, gy;
    // 只有在地图范围内的障碍物才处理
    if (world_to_grid(obs.position.x(), obs.position.y(), gx, gy))
    {
      // 障碍物本身半径的栅格化
      int obs_r_cells = static_cast<int>(std::ceil(obs.radius / resolution));
      int total_r_cells = obs_r_cells + expansion_cells;

      // 简单的方形膨胀，效率高
      for (int dx = -total_r_cells; dx <= total_r_cells; ++dx)
      {
        for (int dy = -total_r_cells; dy <= total_r_cells; ++dy)
        {
          int nx = gx + dx;
          int ny = gy + dy;
          if (nx >= 0 && nx < 200 && ny >= 0 && ny < 200)
          {
            // 简单的圆形判断
            if (dx * dx + dy * dy <= total_r_cells * total_r_cells)
              cells[nx][ny] = 100;
          }
        }
      }
    }
  }
}

// ============================================================================
// A* 算法实现 (修复起点终点被占用时的崩溃问题)
// ============================================================================
int astar_plan(
    const OccupancyGrid2D &grid,
    float start_x, float start_y,
    float goal_x, float goal_y,
    float *path_x, float *path_y,
    int max_points)
{
  int start_gx, start_gy, goal_gx, goal_gy;

  // 1. 边界检查
  if (!grid.world_to_grid(start_x, start_y, start_gx, start_gy))
  {
    ROS_ERROR("[A*] 起点在地图外");
    return 0;
  }
  if (!grid.world_to_grid(goal_x, goal_y, goal_gx, goal_gy))
  {
    ROS_WARN("[A*] 目标在地图外，尝试钳位");
    // 简单的钳位逻辑略，直接返回失败更安全
    return 0;
  }

  // 2. 起点/终点被占用时的最近邻搜索 (BFS)
  auto find_free_neighbor = [&](int &cx, int &cy)
  {
    if (!grid.is_occupied(cx, cy))
      return true;
    std::queue<std::pair<int, int>> q;
    q.push({cx, cy});
    bool visited[200][200] = {false};
    visited[cx][cy] = true;
    int search_r = 10; // 搜索半径 1m

    while (!q.empty())
    {
      auto curr = q.front();
      q.pop();
      if (!grid.is_occupied(curr.first, curr.second))
      {
        cx = curr.first;
        cy = curr.second;
        return true;
      }
      if (std::abs(curr.first - cx) > search_r)
        continue;

      int dx[] = {1, -1, 0, 0};
      int dy[] = {0, 0, 1, -1};
      for (int i = 0; i < 4; ++i)
      {
        int nx = curr.first + dx[i];
        int ny = curr.second + dy[i];
        if (nx >= 0 && nx < 200 && ny >= 0 && ny < 200 && !visited[nx][ny])
        {
          visited[nx][ny] = true;
          q.push({nx, ny});
        }
      }
    }
    return false;
  };

  if (grid.is_occupied(start_gx, start_gy))
  {
    ROS_WARN("[A*] 起点被占用，搜索最近空闲点...");
    if (!find_free_neighbor(start_gx, start_gy))
    {
      ROS_ERROR("[A*] 起点附近无路");
      return 0;
    }
  }
  if (grid.is_occupied(goal_gx, goal_gy))
  {
    ROS_WARN("[A*] 终点被占用，搜索最近空闲点...");
    if (!find_free_neighbor(goal_gx, goal_gy))
    {
      ROS_ERROR("[A*] 终点附近无路");
      return 0;
    }
  }

  // 3. 标准 A*
  auto cmp = [](const std::pair<int, int> &a, const std::pair<int, int> &b)
  { return a.first > b.first; };
  std::priority_queue<std::pair<int, int>, std::vector<std::pair<int, int>>, decltype(cmp)> open_set(cmp);

  // 使用一维数组代替 map 提高效率
  static int g_score[40000];
  static int parent[40000];
  std::fill(g_score, g_score + 40000, 1e9);
  std::fill(parent, parent + 40000, -1);

  int start_id = start_gx * 200 + start_gy;
  int goal_id = goal_gx * 200 + goal_gy;

  g_score[start_id] = 0;
  open_set.push({0, start_id}); // cost, index

  bool found = false;
  int iter = 0;

  while (!open_set.empty() && iter++ < 5000)
  {
    auto curr = open_set.top();
    open_set.pop();
    int curr_id = curr.second;

    if (curr.first > g_score[curr_id] + 200)
      continue; // 简单的 lazy delete

    if (curr_id == goal_id)
    {
      found = true;
      break;
    }

    int cx = curr_id / 200;
    int cy = curr_id % 200;

    int dx[] = {1, -1, 0, 0};
    int dy[] = {0, 0, 1, -1};

    for (int i = 0; i < 4; ++i)
    {
      int nx = cx + dx[i];
      int ny = cy + dy[i];

      if (nx < 0 || nx >= 200 || ny < 0 || ny >= 200)
        continue;
      if (grid.cells[nx][ny] > 50)
        continue;

      int next_id = nx * 200 + ny;
      int new_g = g_score[curr_id] + 1; // 假设权重 1

      if (new_g < g_score[next_id])
      {
        g_score[next_id] = new_g;
        parent[next_id] = curr_id;
        int h = std::abs(nx - goal_gx) + std::abs(ny - goal_gy);
        open_set.push({new_g + h, next_id});
      }
    }
  }

  if (!found)
  {
    ROS_ERROR("[A*] 寻路失败");
    return 0;
  }

  // 4. 路径回溯
  std::vector<std::pair<int, int>> path;
  int curr = goal_id;
  while (curr != -1)
  {
    path.push_back({curr / 200, curr % 200});
    curr = parent[curr];
  }
  std::reverse(path.begin(), path.end());

  int count = 0;
  for (const auto &p : path)
  {
    if (count >= max_points)
      break;
    float wx, wy;
    grid.grid_to_world(p.first, p.second, wx, wy);
    // 转为相对于起飞点的坐标，因为外部使用的是相对坐标
    path_x[count] = wx - init_position_x_take_off;
    path_y[count] = wy - init_position_y_take_off;
    count++;
  }
  return count;
}

// ============================================================================
// 走廊生成器 (保留逻辑，增加健壮性)
// ============================================================================
int generate_corridor(
    const float *astar_path_x, const float *astar_path_y, int num_points,
    float base_width, float *corridor_x, float *corridor_y, float *corridor_width, int max_size)
{
  if (num_points < 2)
    return 0;

  int count = 0;
  for (int i = 0; i < num_points; ++i)
  {
    if (count >= max_size)
      break;
    // 简单的线性复制，实际可以增加平滑
    corridor_x[count] = astar_path_x[i] + init_position_x_take_off; // 转绝对坐标
    corridor_y[count] = astar_path_y[i] + init_position_y_take_off;
    corridor_width[count] = base_width;
    count++;
  }
  return count;
}

// ============================================================================
// VFH+ 避障 (优化：去除了复杂的历史判断，保留核心力场)
// ============================================================================
bool vfh_plus_with_corridor(
    float target_x_rel, float target_y_rel, float target_yaw,
    float uav_radius, float safe_margin, float max_speed, float min_safe_distance,
    const float *corridor_x, const float *corridor_y, const float *corridor_width, int corridor_size,
    bool enable_corridor, bool &out_need_replan)
{
  out_need_replan = false;

  // 坐标转换
  float drone_x = local_pos.pose.pose.position.x;
  float drone_y = local_pos.pose.pose.position.y;
  float drone_yaw = yaw;
  float target_x_world = init_position_x_take_off + target_x_rel;
  float target_y_world = init_position_y_take_off + target_y_rel;

  float dx = target_x_world - drone_x;
  float dy = target_y_world - drone_y;
  float dist_target = std::sqrt(dx * dx + dy * dy);

  // 1. 到达判断
  if (dist_target < 0.3f)
  {
    setpoint_raw.position.x = drone_x;
    setpoint_raw.position.y = drone_y;
    setpoint_raw.yaw = target_yaw;
    return true;
  }

  // 2. 直方图构建 (Polar Histogram)
  const int HIST_SIZE = 72;
  float histogram[HIST_SIZE] = {0};

  for (const auto &obs : obstacles)
  {
    float ox = obs.position.x() - drone_x;
    float oy = obs.position.y() - drone_y;
    float dist = std::sqrt(ox * ox + oy * oy);

    if (dist > 3.0f || dist < 0.01f)
      continue; // 感知范围

    float angle = std::atan2(oy, ox) - drone_yaw;
    while (angle > M_PI)
      angle -= 2 * M_PI;
    while (angle < -M_PI)
      angle += 2 * M_PI;

    // 障碍物张角
    float obs_w = std::asin(std::min(1.0f, (obs.radius + safe_margin) / dist));
    int bin_center = static_cast<int>((angle + M_PI) / (2 * M_PI) * HIST_SIZE) % HIST_SIZE;
    int bin_width = static_cast<int>(obs_w / (2 * M_PI) * HIST_SIZE) + 1;

    for (int k = bin_center - bin_width; k <= bin_center + bin_width; ++k)
    {
      int idx = (k + HIST_SIZE) % HIST_SIZE;
      histogram[idx] += 10.0f / dist; // 距离越近，代价越大
    }
  }

  // 3. 代价函数选择最佳方向
  float target_angle = std::atan2(dy, dx) - drone_yaw;
  while (target_angle > M_PI)
    target_angle -= 2 * M_PI;
  while (target_angle < -M_PI)
    target_angle += 2 * M_PI;

  int best_bin = -1;
  float min_cost = 1e9;

  for (int i = 0; i < HIST_SIZE; ++i)
  {
    if (histogram[i] > 15.0f)
      continue; // 阈值，太高视为不可行

    float bin_angle = -M_PI + i * (2 * M_PI / HIST_SIZE) + (M_PI / HIST_SIZE) * 0.5f;
    float cost = std::abs(bin_angle - target_angle) + histogram[i] * 0.1f;

    if (cost < min_cost)
    {
      min_cost = cost;
      best_bin = i;
    }
  }

  if (best_bin == -1)
  {
    out_need_replan = true;
    // 死锁时原地悬停
    setpoint_raw.position.x = drone_x;
    setpoint_raw.position.y = drone_y;
    setpoint_raw.yaw = drone_yaw;
    ROS_WARN_THROTTLE(1.0, "[VFH] 陷入死锁，请求重规划");
    return false;
  }

  float move_angle = -M_PI + best_bin * (2 * M_PI / HIST_SIZE) + drone_yaw;

  // 4. 走廊拉力 (简单版)
  if (enable_corridor && corridor_size > 0)
  {
    // 找最近点
    int nearest_idx = -1;
    float min_d = 1e9;
    for (int i = 0; i < corridor_size; ++i)
    {
      float d = std::sqrt(std::pow(corridor_x[i] - drone_x, 2) + std::pow(corridor_y[i] - drone_y, 2));
      if (d < min_d)
      {
        min_d = d;
        nearest_idx = i;
      }
    }

    if (nearest_idx != -1 && min_d > corridor_width[nearest_idx] / 2.0f)
    {
      float cx = corridor_x[nearest_idx] - drone_x;
      float cy = corridor_y[nearest_idx] - drone_y;
      float corr_angle = std::atan2(cy, cx);

      // 简单的向量合成：VFH方向 + 走廊回拉方向
      float w_c = 0.5f; // 走廊权重
      move_angle = move_angle * (1 - w_c) + corr_angle * w_c;
    }
  }

  // 5. 生成控制量
  float speed = std::min(max_speed, dist_target);
  if (std::abs(move_angle - drone_yaw) > 0.5)
    speed *= 0.5f; // 转弯减速

  setpoint_raw.position.x = drone_x + std::cos(move_angle) * speed * 0.5f; // 0.5s 预测
  setpoint_raw.position.y = drone_y + std::sin(move_angle) * speed * 0.5f;
  setpoint_raw.position.z = ALTITUDE + init_position_z_take_off;
  setpoint_raw.yaw = move_angle;

  return false;
}

// ============================================================================
// 回调与辅助函数
// ============================================================================
void state_cb(const mavros_msgs::State::ConstPtr &msg)
{
  mavros_connection_state = *msg;
}

void local_pos_cb(const nav_msgs::Odometry::ConstPtr &msg)
{
  local_pos = *msg;
  tf::quaternionMsgToTF(local_pos.pose.pose.orientation, quat);
  tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

  // 首次记录起飞点 (高度大于 0.1m 且未记录)
  if (!flag_init_position && local_pos.pose.pose.position.z > 0.1)
  {
    init_position_x_take_off = local_pos.pose.pose.position.x;
    init_position_y_take_off = local_pos.pose.pose.position.y;
    init_position_z_take_off = local_pos.pose.pose.position.z;
    init_yaw_take_off = yaw;
    flag_init_position = true;
    ROS_INFO("起飞点已记录: (%.2f, %.2f)", init_position_x_take_off, init_position_y_take_off);
  }
}

bool mission_pos_cruise(float x, float y, float z, float target_yaw, float error_max)
{
  setpoint_raw.type_mask = mavros_msgs::PositionTarget::IGNORE_VX | mavros_msgs::PositionTarget::IGNORE_VY | mavros_msgs::PositionTarget::IGNORE_VZ |
                           mavros_msgs::PositionTarget::IGNORE_AFX | mavros_msgs::PositionTarget::IGNORE_AFY | mavros_msgs::PositionTarget::IGNORE_AFZ |
                           mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
  setpoint_raw.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
  setpoint_raw.position.x = x + init_position_x_take_off;
  setpoint_raw.position.y = y + init_position_y_take_off;
  setpoint_raw.position.z = z + init_position_z_take_off;
  setpoint_raw.yaw = target_yaw;

  float dx = local_pos.pose.pose.position.x - setpoint_raw.position.x;
  float dy = local_pos.pose.pose.position.y - setpoint_raw.position.y;
  float dz = local_pos.pose.pose.position.z - setpoint_raw.position.z;

  if (std::sqrt(dx * dx + dy * dy) < error_max && std::abs(dz) < error_max)
    return true;
  return false;
}

bool precision_land(float err_max)
{
  // 简化版降落：直接降
  setpoint_raw.position.x = local_pos.pose.pose.position.x;
  setpoint_raw.position.y = local_pos.pose.pose.position.y;
  setpoint_raw.position.z = local_pos.pose.pose.position.z - 0.15; // 缓降

  if (local_pos.pose.pose.position.z < init_position_z_take_off + 0.15)
    return true;
  return false;
}

// ============================================================================
// 主函数 (修复解锁逻辑)
// ============================================================================
int main(int argc, char **argv)
{
  setlocale(LC_ALL, "");
  ros::init(argc, argv, "astar_vfh_avoidance");
  ros::NodeHandle nh;

  ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
  ros::Subscriber local_pos_sub = nh.subscribe<nav_msgs::Odometry>("/mavros/local_position/odom", 10, local_pos_cb);
  ros::Subscriber livox_sub = nh.subscribe<livox_ros_driver::CustomMsg>("/livox/lidar", 10, livox_cb_wrapper);
  mavros_setpoint_pos_pub = nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 10);

  ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
  ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

  ros::Rate rate(20.0);

  // 参数加载
  nh.param<float>("target_x", target_x, 5.0f);
  nh.param<float>("target_y", target_y, 0.0f);
  print_param();

  // 等待连接
  while (ros::ok() && !mavros_connection_state.connected)
  {
    ros::spinOnce();
    rate.sleep();
    ROS_INFO_THROTTLE(2.0, "等待 MAVROS 连接...");
  }

  // 初始化 Setpoint 为当前位置 (防止切模式时飞丢)
  // 注意：在没有 local_pos 之前，不要发 0,0,0
  while (ros::ok() && local_pos.header.seq == 0)
  {
    ros::spinOnce();
    rate.sleep();
    ROS_INFO_THROTTLE(1.0, "等待 GPS/Odom 初始化...");
  }

  // 预发送 Setpoint (PX4 要求)
  setpoint_raw.type_mask = mavros_msgs::PositionTarget::IGNORE_VX | mavros_msgs::PositionTarget::IGNORE_VY | mavros_msgs::PositionTarget::IGNORE_VZ |
                           mavros_msgs::PositionTarget::IGNORE_AFX | mavros_msgs::PositionTarget::IGNORE_AFY | mavros_msgs::PositionTarget::IGNORE_AFZ |
                           mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
  setpoint_raw.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
  setpoint_raw.position.x = local_pos.pose.pose.position.x;
  setpoint_raw.position.y = local_pos.pose.pose.position.y;
  setpoint_raw.position.z = local_pos.pose.pose.position.z; // 保持当前高度，哪怕是地面
  setpoint_raw.yaw = yaw;

  for (int i = 50; ros::ok() && i > 0; --i)
  {
    mavros_setpoint_pos_pub.publish(setpoint_raw);
    ros::spinOnce();
    rate.sleep();
  }

  ROS_INFO("准备就绪，开始任务循环");
  ros::Time last_req = ros::Time::now();
  mission_num = 0; // 0: 解锁起飞阶段

  while (ros::ok())
  {
    // === 核心修复：始终发布 setpoint，否则 OFFBOARD 会掉 ===
    mavros_setpoint_pos_pub.publish(setpoint_raw);

    // 状态机
    switch (mission_num)
    {
    case 0: // 准备与起飞
    {
      // 保持当前位置 (直到起飞命令)
      if (mavros_connection_state.mode != "OFFBOARD" && (ros::Time::now() - last_req > ros::Duration(5.0)))
      {
        mavros_msgs::SetMode offb_set_mode;
        offb_set_mode.request.custom_mode = "OFFBOARD";
        if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
        {
          ROS_INFO("OFFBOARD 模式已请求");
        }
        last_req = ros::Time::now();
      }
      else if (mavros_connection_state.mode == "OFFBOARD" && !mavros_connection_state.armed && (ros::Time::now() - last_req > ros::Duration(5.0)))
      {
        mavros_msgs::CommandBool arm_cmd;
        arm_cmd.request.value = true;
        if (arming_client.call(arm_cmd) && arm_cmd.response.success)
        {
          ROS_INFO("解锁成功！");
        }
        last_req = ros::Time::now();
      }

      if (mavros_connection_state.mode == "OFFBOARD" && mavros_connection_state.armed)
      {
        // 已解锁，给起飞高度
        // 如果还没有记录起飞点，现在记录（作为双重保险）
        if (!flag_init_position && local_pos.pose.pose.position.z > -0.5)
        { // 地面也算
          init_position_x_take_off = local_pos.pose.pose.position.x;
          init_position_y_take_off = local_pos.pose.pose.position.y;
          init_position_z_take_off = local_pos.pose.pose.position.z;
          flag_init_position = true;
        }

        // 执行起飞
        setpoint_raw.position.x = init_position_x_take_off;
        setpoint_raw.position.y = init_position_y_take_off;
        setpoint_raw.position.z = init_position_z_take_off + ALTITUDE;

        if (local_pos.pose.pose.position.z > init_position_z_take_off + ALTITUDE - 0.15)
        {
          ROS_INFO("起飞完成，切换到任务 1 (巡航到目标附近)");
          mission_num = 1;
        }
      }
      else
      {
        // 未解锁前，持续发送当前位置，防止漂移
        setpoint_raw.position.x = local_pos.pose.pose.position.x;
        setpoint_raw.position.y = local_pos.pose.pose.position.y;
        setpoint_raw.position.z = local_pos.pose.pose.position.z;
      }
      break;
    }

    case 1: // 初始巡航 / 触发避障
    {
      // 尝试直接飞向目标
      bool reached = mission_pos_cruise(target_x, target_y, ALTITUDE, 0, err_max);

      // 此处应该加入触发避障的逻辑
      // 例如：如果发现前方有障碍，切换到 mission_num = 2
      // 简单起见，这里直接切 mission 2 进行规划
      mission_num = 2;
      avoidance_state = PLANNING;
      state_start_time = ros::Time::now();
      ROS_INFO("进入避障规划阶段");
      break;
    }

    case 2: // 避障状态机
    {
      switch (avoidance_state)
      {
      case PLANNING:
      {
        OccupancyGrid2D grid;
        grid.update_with_obstacles(obstacles, UAV_radius, safe_margin); // 这里的 obstacles 来自 detect_obs 的全局变量

        path_size = astar_plan(grid,
                               local_pos.pose.pose.position.x, local_pos.pose.pose.position.y,
                               init_position_x_take_off + target_x, init_position_y_take_off + target_y,
                               astar_path_x, astar_path_y, MAX_PATH_POINTS);

        if (path_size > 0)
        {
          corridor_size = generate_corridor(astar_path_x, astar_path_y, path_size, CORRIDOR_WIDTH_BASE, corridor_x, corridor_y, corridor_width, MAX_CORRIDOR_POINTS);
          avoidance_state = AVOIDING;
          ROS_INFO("规划成功，开始避障飞行");
        }
        else
        {
          // 规划失败，原地悬停重试
          ROS_WARN_THROTTLE(1.0, "规划失败，重试中...");
          setpoint_raw.position.x = local_pos.pose.pose.position.x;
          setpoint_raw.position.y = local_pos.pose.pose.position.y;
        }
        break;
      }
      case AVOIDING:
      {
        bool need_replan = false;
        vfh_plus_with_corridor(target_x, target_y, 0, UAV_radius, safe_margin, MAX_SPEED, MIN_SAFE_DISTANCE, corridor_x, corridor_y, corridor_width, corridor_size, ENABLE_CORRIDOR, need_replan);

        if (need_replan)
        {
          avoidance_state = REPLANNING;
          last_replan_time = ros::Time::now();
        }

        // 简单判定到达
        float dist_to_target = std::sqrt(std::pow(local_pos.pose.pose.position.x - (init_position_x_take_off + target_x), 2) +
                                         std::pow(local_pos.pose.pose.position.y - (init_position_y_take_off + target_y), 2));
        if (dist_to_target < 0.3)
        {
          ROS_INFO("到达目标，准备降落");
          mission_num = 3;
        }
        break;
      }
      case REPLANNING:
      {
        if (ros::Time::now() - last_replan_time > ros::Duration(REPLAN_COOLDOWN))
        {
          avoidance_state = PLANNING;
        }
        else
        {
          // 冷却期悬停或减速
          setpoint_raw.position.x = local_pos.pose.pose.position.x;
          setpoint_raw.position.y = local_pos.pose.pose.position.y;
        }
        break;
      }
      }
      break;
    }

    case 3: // 降落
    {
      if (precision_land(0.2))
      {
        ROS_INFO("降落完成，任务结束");
        mission_num = -1; // 结束
      }
      break;
    }
    }

    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}