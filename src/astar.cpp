#include <atsar.h>

// 全局变量定义
int mission_num = 0;
float if_debug = 0;
float err_max = 0.2;
const float HOVER_DURATION = 10.0f;
float hold_flag = false;

// 任务参数
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

// 重规划参数
int FRONT_OBSTACLE_TH = 2;
int BACK_OBSTACLE_TH = 1;
float DETECTION_RADIUS = 1.5f;
float REPLAN_COOLDOWN = 5.0f; // 重规划冷却时间（秒）

// ========== 状态机定义 ==========
enum MissionState
{
  INIT,       // 初始化
  TAKEOFF,    // 起飞
  PLANNING,   // A*全局规划
  AVOIDING,   // VFH+避障中
  REPLANNING, // A*重规划中
  LANDING,    // 降落
  EMERGENCY   // 紧急悬停
};

MissionState current_state = INIT;
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

void print_param()
{
  std::cout << "=== 避障系统参数 ===" << std::endl;
  std::cout << "UAV_radius: " << UAV_radius << " m" << std::endl;
  std::cout << "safe_margin: " << safe_margin << " m" << std::endl;
  std::cout << "MAX_SPEED: " << MAX_SPEED << " m/s" << std::endl;
  std::cout << "CORRIDOR_WIDTH_BASE: " << CORRIDOR_WIDTH_BASE << " m" << std::endl;
  std::cout << "FRONT_OBSTACLE_TH: " << FRONT_OBSTACLE_TH << std::endl;
  std::cout << "REPLAN_COOLDOWN: " << REPLAN_COOLDOWN << " s" << std::endl;
}

int main(int argc, char **argv)
{
  setlocale(LC_ALL, "");
  ros::init(argc, argv, "VFF");
  ros::NodeHandle nh;

  // 订阅话题
  ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
  ros::Subscriber local_pos_sub = nh.subscribe<nav_msgs::Odometry>("/mavros/local_position/odom", 10, local_pos_cb);
  ros::Publisher mavros_setpoint_pos_pub = nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 100);
  ros::Subscriber livox_sub = nh.subscribe<livox_ros_driver::CustomMsg>("/livox/lidar", 10, livox_cb_wrapper);

  // 服务客户端
  ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
  ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

  ros::Rate rate(20);

  // ========== 参数读取 ==========
  nh.param<float>("err_max", err_max, 0);
  nh.param<float>("if_debug", if_debug, 0);
  nh.param<float>("target_x", target_x, 5.0f);
  nh.param<float>("target_y", target_y, 0.0f);
  nh.param<float>("target_yaw", target_yaw, 0.0f);
  nh.param<float>("UAV_radius", UAV_radius, 0.3f);
  nh.param<float>("time_final", time_final, 70.0f);

  nh.param<float>("safe_margin", safe_margin, 0.4f);
  nh.param<float>("MAX_SPEED", MAX_SPEED, 0.9f);
  nh.param<float>("MIN_SAFE_DISTANCE", MIN_SAFE_DISTANCE, 0.25f);
  nh.param<float>("CORRIDOR_WIDTH_BASE", CORRIDOR_WIDTH_BASE, 0.8f);
  nh.param<bool>("ENABLE_CORRIDOR", ENABLE_CORRIDOR, true);

  nh.param<int>("FRONT_OBSTACLE_TH", FRONT_OBSTACLE_TH, 2);
  nh.param<int>("BACK_OBSTACLE_TH", BACK_OBSTACLE_TH, 1);
  nh.param<float>("DETECTION_RADIUS", DETECTION_RADIUS, 1.5f);
  nh.param<float>("REPLAN_COOLDOWN", REPLAN_COOLDOWN, 5.0f);

  print_param();

  int choice = 0;
  std::cout << "\n1 to go on , else to quit: ";
  std::cin >> choice;
  if (choice != 1)
    return 0;
  ros::spinOnce();
  rate.sleep();

  // 等待飞控连接
  while (ros::ok() && !current_state.connected)
  {
    ros::spinOnce();
    rate.sleep();
  }

  // 发送初始setpoint
  setpoint_raw.type_mask = 64 + 128 + 256 + 512 + 2048;
  setpoint_raw.coordinate_frame = 1;
  setpoint_raw.position.x = 0;
  setpoint_raw.position.y = 0;
  setpoint_raw.position.z = ALTITUDE;
  setpoint_raw.yaw = 0;

  for (int i = 100; ros::ok() && i > 0; --i)
  {
    mavros_setpoint_pos_pub.publish(setpoint_raw);
    ros::spinOnce();
    rate.sleep();
  }
  std::cout << "OK: Setpoints sent, ready for OFFBOARD" << std::endl;

  mavros_msgs::SetMode offb_set_mode;
  offb_set_mode.request.custom_mode = "OFFBOARD";
  mavros_msgs::CommandBool arm_cmd;
  arm_cmd.request.value = true;
  ros::Time last_request = ros::Time::now();

  // ========== 状态机主循环 ==========
  while (ros::ok())
  {
    ROS_WARN("[STATE] Current state: %d (%s), mission_num=%d",
             current_state,
             (current_state == INIT ? "INIT" : current_state == TAKEOFF  ? "TAKEOFF"
                                           : current_state == PLANNING   ? "PLANNING"
                                           : current_state == AVOIDING   ? "AVOIDING"
                                           : current_state == REPLANNING ? "REPLANNING"
                                           : current_state == LANDING    ? "LANDING"
                                                                         : "EMERGENCY"),
             mission_num);

    switch (current_state)
    {
    case INIT:
      // 初始化：等待起飞高度
      if (fabs(local_pos.pose.pose.position.z - ALTITUDE) < 0.2)
      {
        current_state = TAKEOFF;
        state_start_time = ros::Time::now();
        ROS_INFO("[STATE] Switched to TAKEOFF");
      }
      mission_pos_cruise(0, 0, ALTITUDE, 0, err_max);
      break;

    case TAKEOFF:
      // 起飞：悬停3秒
      if (mission_pos_cruise(0, 0, ALTITUDE, 0, err_max))
      {
        current_state = PLANNING;
        state_start_time = ros::Time::now();
        ROS_INFO("[STATE] Switched to PLANNING");
      }
      else if ((ros::Time::now() - state_start_time).toSec() > 3.0)
      {
        current_state = PLANNING;
        state_start_time = ros::Time::now();
        ROS_INFO("[STATE] Switched to PLANNING (timeout)");
      }
      break;

    case PLANNING:
      // A*全局规划
      {
        OccupancyGrid2D grid;
        grid.update_with_obstacles(obstacles, UAV_radius, safe_margin);

        float goal_x_world = init_position_x_take_off + target_x;
        float goal_y_world = init_position_y_take_off + target_y;

        path_size = astar_plan(
            grid,
            local_pos.pose.pose.position.x,
            local_pos.pose.pose.position.y,
            goal_x_world,
            goal_y_world,
            astar_path_x,
            astar_path_y,
            MAX_PATH_POINTS);

        if (path_size > 0)
        {
          // 生成走廊
          corridor_size = generate_corridor(
              astar_path_x, astar_path_y,
              path_size,
              CORRIDOR_WIDTH_BASE,
              corridor_x, corridor_y,
              corridor_width,
              MAX_CORRIDOR_POINTS);

          if (corridor_size > 0)
          {
            current_state = AVOIDING;
            state_start_time = ros::Time::now();
            ROS_INFO("[STATE] Switched to AVOIDING (path_size=%d, corridor_size=%d)",
                     path_size, corridor_size);
          }
          else
          {
            ROS_ERROR("[STATE] Corridor generation failed!");
            current_state = EMERGENCY;
          }
        }
        else
        {
          ROS_ERROR("[STATE] A* planning failed!");
          current_state = EMERGENCY;
        }
      }
      break;

    case AVOIDING:
      // VFH+避障主循环
      {
        bool need_replan = false;
        bool reached = vfh_plus_with_corridor(
            target_x, target_y, target_yaw,
            UAV_radius, safe_margin, MAX_SPEED, MIN_SAFE_DISTANCE,
            corridor_x, corridor_y, corridor_width, corridor_size,
            ENABLE_CORRIDOR,
            need_replan);

        // 定时跳转：超时保护
        ros::Duration elapsed = ros::Time::now() - state_start_time;
        if (elapsed.toSec() > time_final)
        {
          ROS_WARN("[STATE] Mission timeout (%.1fs), switching to LANDING", elapsed.toSec());
          current_state = LANDING;
          state_start_time = ros::Time::now();
          break;
        }

        // 重规划触发
        if (need_replan && (ros::Time::now() - last_replan_time).toSec() > REPLAN_COOLDOWN)
        {
          ROS_WARN("[STATE] Replanning triggered!");
          current_state = REPLANNING;
          state_start_time = ros::Time::now();
          last_replan_time = ros::Time::now();
          break;
        }

        // 到达目标
        if (reached)
        {
          ROS_WARN("[STATE] Target reached!");
          current_state = LANDING;
          state_start_time = ros::Time::now();
        }
      }
      break;

    case REPLANNING:
      // A*重规划（同PLANNING，但使用当前位置作为起点）
      {
        OccupancyGrid2D grid;
        grid.update_with_obstacles(obstacles, UAV_radius, safe_margin);

        float goal_x_world = init_position_x_take_off + target_x;
        float goal_y_world = init_position_y_take_off + target_y;

        path_size = astar_plan(
            grid,
            local_pos.pose.pose.position.x, // 当前位置作为新起点
            local_pos.pose.pose.position.y,
            goal_x_world,
            goal_y_world,
            astar_path_x,
            astar_path_y,
            MAX_PATH_POINTS);

        if (path_size > 0)
        {
          corridor_size = generate_corridor(
              astar_path_x, astar_path_y,
              path_size,
              CORRIDOR_WIDTH_BASE,
              corridor_x, corridor_y,
              corridor_width,
              MAX_CORRIDOR_POINTS);

          if (corridor_size > 0)
          {
            current_state = AVOIDING;
            state_start_time = ros::Time::now();
            ROS_INFO("[STATE] Replanning success, switched to AVOIDING");
          }
          else
          {
            ROS_ERROR("[STATE] Corridor generation failed after replanning!");
            current_state = EMERGENCY;
          }
        }
        else
        {
          ROS_ERROR("[STATE] A* replanning failed!");
          // 降级：禁用走廊，仅用VFH+避障
          ENABLE_CORRIDOR = false;
          current_state = AVOIDING;
          state_start_time = ros::Time::now();
          ROS_WARN("[STATE] Degraded to VFH+ only (no corridor)");
        }
      }
      break;

    case LANDING:
      // 降落
      if (precision_land(err_max))
      {
        ROS_WARN("[STATE] Landing complete, mission success!");
        mission_num = -1; // 任务结束
      }
      break;

    case EMERGENCY:
      // 紧急悬停
      setpoint_raw.position.x = local_pos.pose.pose.position.x;
      setpoint_raw.position.y = local_pos.pose.pose.position.y;
      setpoint_raw.position.z = ALTITUDE;
      setpoint_raw.yaw = target_yaw;

      ROS_ERROR("[STATE] EMERGENCY MODE: Hovering at current position");
      // 人工干预后恢复
      if (mission_num == -1)
      {
        exit(0);
      }
      break;
    }

    // 发布控制指令
    mavros_setpoint_pos_pub.publish(setpoint_raw);
    ros::spinOnce();
    rate.sleep();

    if (mission_num == -1)
    {
      exit(0);
    }
  }
  return 0;
}