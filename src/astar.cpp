#include <astar.h>

// 全局变量定义
int mission_num = 0;
float if_debug = 0;
float err_max = 0.2;
const float HOVER_DURATION = 10.0f; // 降落悬停时间（秒）
float hold_flag = false;

// 【可配置参数】（全部从yaml读取）
float target_x = 5.0f;     // 目标点x（相对起飞点，米）
float target_y = 0.0f;     // 目标点y（相对起飞点，米）
float target_z = ALTITUDE; // 目标点z高度（米）
float target_yaw = 0.0f;   // 目标偏航角（弧度）
float UAV_radius = 0.3f;   // 无人机等效半径（米）
float time_final = 70.0f;  // 超时时间（秒）

// 避障参数（全部从yaml读取）
float safe_margin = 0.4f;        // 安全裕度（米）
float MAX_SPEED = 0.9f;          // 最大前进速度（米/秒）
float MIN_SAFE_DISTANCE = 0.25f; // 力场计算最小距离（防除零）

// ========== 新增：A* + 走廊 + VFH+ 参数 ==========
float CORRIDOR_WIDTH_BASE = 0.8f; // 基础走廊宽度（米）
bool ENABLE_CORRIDOR = true;      // 是否启用走廊约束

// 重规划参数
float REPLAN_COOLDOWN = 5.0f; // 重规划冷却时间（秒）

// ========== 任务状态机（仅用于 mission2 内部） ==========
enum AvoidanceState
{
  PLANNING,      // A*全局规划
  AVOIDING,      // VFH+避障中
  REPLANNING,    // A*重规划中
  TARGET_REACHED // 目标已到达
};

AvoidanceState avoidance_state = PLANNING; // mission2内部状态机
ros::Time state_start_time;
ros::Time last_replan_time;

// 路径与走廊数据（mission2专用）
const int MAX_PATH_POINTS = 200;
float astar_path_x[MAX_PATH_POINTS] = {0};
float astar_path_y[MAX_PATH_POINTS] = {0};
int path_size = 0;

const int MAX_CORRIDOR_POINTS = 200;
float corridor_x[MAX_CORRIDOR_POINTS] = {0};
float corridor_y[MAX_CORRIDOR_POINTS] = {0};
float corridor_width[MAX_CORRIDOR_POINTS] = {0};
int corridor_size = 0;

// 重规划参数
float REPLAN_COOLDOWN = 5.0f; // 重规划冷却时间（秒）

void print_param()
{
  std::cout << "=== 避障系统参数 ===" << std::endl;
  std::cout << "UAV_radius: " << UAV_radius << " m" << std::endl;
  std::cout << "safe_margin: " << safe_margin << " m" << std::endl;
  std::cout << "MAX_SPEED: " << MAX_SPEED << " m/s" << std::endl;
  std::cout << "CORRIDOR_WIDTH_BASE: " << CORRIDOR_WIDTH_BASE << " m" << std::endl;
  std::cout << "REPLAN_COOLDOWN: " << REPLAN_COOLDOWN << " s" << std::endl;
}

int main(int argc, char **argv)
{
  // 防止中文输出乱码
  setlocale(LC_ALL, "");

  // 初始化ROS节点
  ros::init(argc, argv, "VFF");
  ros::NodeHandle nh;

  // 订阅mavros相关话题
  ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
  ros::Subscriber local_pos_sub = nh.subscribe<nav_msgs::Odometry>("/mavros/local_position/odom", 10, local_pos_cb);

  // 发布无人机多维控制话题
  ros::Publisher mavros_setpoint_pos_pub = nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 100);
  ros::Subscriber livox_sub = nh.subscribe<livox_ros_driver::CustomMsg>("/livox/lidar", 10, livox_cb_wrapper);

  // 创建服务客户端
  ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
  ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
  ros::ServiceClient ctrl_pwm_client = nh.serviceClient<mavros_msgs::CommandLong>("mavros/cmd/command");

  // 设置话题发布频率，需要大于2Hz，飞控连接有500ms的心跳包
  ros::Rate rate(20);

  // ========== 参数读取（yaml → 全局变量） ==========
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
  nh.param<float>("REPLAN_COOLDOWN", REPLAN_COOLDOWN, 5.0f);

  print_param();

  int choice = 0;
  std::cout << "1 to go on , else to quit" << std::endl;
  std::cin >> choice;
  if (choice != 1)
    return 0;
  ros::spinOnce();
  rate.sleep();

  // ========== 起飞前准备（不纳入状态机） ==========
  // 等待连接到飞控
  while (ros::ok() && !mavros_connection_state.connected)
  {
    ros::spinOnce();
    rate.sleep();
  }

  // 设置初始setpoint
  setpoint_raw.type_mask = /*1 + 2 + 4 + 8 + 16 + 32*/ +64 + 128 + 256 + 512 /*+ 1024 + 2048*/;
  setpoint_raw.coordinate_frame = 1;
  setpoint_raw.position.x = 0;
  setpoint_raw.position.y = 0;
  setpoint_raw.position.z = ALTITUDE;
  setpoint_raw.yaw = 0;

  // 发送初始setpoint（100次）
  for (int i = 100; ros::ok() && i > 0; --i)
  {
    mavros_setpoint_pos_pub.publish(setpoint_raw);
    ros::spinOnce();
    rate.sleep();
  }
  std::cout << "ok" << std::endl;

  // OFFBOARD模式与解锁（不纳入状态机）
  mavros_msgs::SetMode offb_set_mode;
  offb_set_mode.request.custom_mode = "OFFBOARD";
  mavros_msgs::CommandBool arm_cmd;
  arm_cmd.request.value = true;
  ros::Time last_request = ros::Time::now();

  while (ros::ok())
  {
    if (mavros_connection_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(3.0)))
    {
      if (if_debug == 1)
      {
        if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
        {
          ROS_INFO("Offboard enabled");
        }
      }
      else
      {
        ROS_INFO("Waiting for OFFBOARD mode");
      }
      last_request = ros::Time::now();
    }
    else
    {
      if (!mavros_connection_state.armed && (ros::Time::now() - last_request > ros::Duration(3.0)))
      {
        if (arming_client.call(arm_cmd) && arm_cmd.response.success)
        {
          ROS_INFO("Vehicle armed");
        }
        last_request = ros::Time::now();
      }
    }

    // 当无人机到达起飞点高度后，悬停3秒后进入任务模式
    if (fabs(local_pos.pose.pose.position.z - ALTITUDE) < 0.2)
    {
      if (ros::Time::now() - last_request > ros::Duration(1.0))
      {
        mission_num = 1; // 进入mission1（起飞）
        last_request = ros::Time::now();
        break;
      }
    }

    mission_pos_cruise(0, 0, ALTITUDE, 0, err_max);
    mavros_setpoint_pos_pub.publish(setpoint_raw);
    ros::spinOnce();
    rate.sleep();
  }

  // ========== 任务主循环（mission_num状态机） ==========
  while (ros::ok())
  {
    ROS_WARN("mission_num = %d", mission_num);

    switch (mission_num)
    {
    // mission1: 起飞（保持原有逻辑不变）
    case 1:
    {
      if (mission_pos_cruise(0, 0, ALTITUDE, 0, err_max))
      {
        mission_num = 2; // 起飞完成，进入避障任务
        last_request = ros::Time::now();
        // 重置mission2内部状态机
        avoidance_state = PLANNING;
        state_start_time = ros::Time::now();
        ROS_INFO("[MISSION2] 进入避障任务，开始A*规划");
      }
      else if (ros::Time::now() - last_request >= ros::Duration(9.0))
      {
        mission_num = 2; // 超时9秒强制进入避障
        last_request = ros::Time::now();
        avoidance_state = PLANNING;
        state_start_time = ros::Time::now();
        ROS_WARN("[MISSION2] 起飞超时，强制进入避障任务");
      }
      break;
    }

    // mission2: 避障任务（新增状态机）
    case 2:
    {
      // ========== mission2内部状态机 ==========
      switch (avoidance_state)
      {
      case PLANNING:
      {
        // A*全局规划
        OccupancyGrid2D grid;
        grid.update_with_obstacles(obstacles, UAV_radius, safe_margin);
        float goal_x_world = init_position_x_take_off + target_x;
        float goal_y_world = init_position_y_take_off + target_y;

        // 【位置】第2122行 - 使用增量A*
        path_size = incremental_astar_plan(
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
            avoidance_state = AVOIDING;
            state_start_time = ros::Time::now();
            ROS_INFO("[MISSION2] A*规划成功，切换到AVOIDING (路径点:%d, 走廊点:%d)",
                     path_size, corridor_size);
          }
          else
          {
            ROS_ERROR("[MISSION2] 走廊生成失败，重试规划");
            // 1秒后重试
            if ((ros::Time::now() - state_start_time).toSec() > 1.0)
            {
              state_start_time = ros::Time::now();
            }
          }
        }
        else
        {
          ROS_ERROR("[MISSION2] A*规划失败，重试规划");
          // 2秒后重试
          if ((ros::Time::now() - state_start_time).toSec() > 2.0)
          {
            state_start_time = ros::Time::now();
          }
        }

        // 超时保护：5秒内未规划成功则降级为纯VFH+
        if ((ros::Time::now() - state_start_time).toSec() > 5.0)
        {
          ROS_WARN("[MISSION2] A*规划超时，降级为纯VFH+避障");
          ENABLE_CORRIDOR = false;
          avoidance_state = AVOIDING;
          state_start_time = ros::Time::now();
        }
        break;
      }
      case AVOIDING:
      {
        // VFH+避障（含走廊软约束）
        bool need_replan = false;
        bool reached = vfh_plus_with_corridor(
            target_x, target_y, target_yaw,
            UAV_radius, safe_margin, MAX_SPEED, MIN_SAFE_DISTANCE,
            corridor_x, corridor_y, corridor_width, corridor_size,
            ENABLE_CORRIDOR,
            need_replan);

        // 定时跳转：任务超时保护
        ros::Duration elapsed = ros::Time::now() - last_request;
        if (elapsed.toSec() > time_final)
        {
          ROS_WARN("[MISSION2] 任务超时(%.1fs)，切换到降落", elapsed.toSec());
          mission_num = 3;
          break;
        }

        // 重规划触发
        if (need_replan && (ros::Time::now() - last_replan_time).toSec() > REPLAN_COOLDOWN)
        {
          ROS_WARN("[MISSION2] 触发重规划（振荡/目标不可达/视野变化）");
          avoidance_state = REPLANNING;
          state_start_time = ros::Time::now();
          last_replan_time = ros::Time::now();
          break;
        }

        // 到达目标
        if (reached)
        {
          ROS_WARN("[MISSION2] ✓ 成功抵达目标点(%.2f, %.2f)!", target_x, target_y);
          avoidance_state = TARGET_REACHED;
          state_start_time = ros::Time::now();
        }
        break;
      }

      case REPLANNING:
      {
        // A*重规划（使用当前位置作为新起点）
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
            avoidance_state = AVOIDING;
            state_start_time = ros::Time::now();
            ROS_INFO("[MISSION2] 重规划成功，切换到AVOIDING");
          }
          else
          {
            // 走廊生成失败：降级为纯VFH+
            ROS_WARN("[MISSION2] 走廊生成失败，降级为纯VFH+");
            ENABLE_CORRIDOR = false;
            avoidance_state = AVOIDING;
            state_start_time = ros::Time::now();
          }
        }
        else
        {
          // A*重规划失败：降级为纯VFH+
          ROS_WARN("[MISSION2] A*重规划失败，降级为纯VFH+");
          ENABLE_CORRIDOR = false;
          avoidance_state = AVOIDING;
          state_start_time = ros::Time::now();
        }
        break;
      }

      case TARGET_REACHED:
      {
        // 等待1秒确保稳定，然后切换到降落
        if ((ros::Time::now() - state_start_time).toSec() > 1.0)
        {
          mission_num = 3;
          ROS_INFO("[MISSION2] 目标稳定，切换到降落任务");
        }
        break;
      }
      }
      break;
    }

    // mission3: 降落（保持原有逻辑不变）
    case 3:
    {
      if (precision_land(err_max))
      {
        ROS_WARN("✓ 精确降落完成，任务结束！");
        mission_num = -1; // 任务结束
      }
      break;
    }
    }

    // 发布控制指令（所有mission共用）
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