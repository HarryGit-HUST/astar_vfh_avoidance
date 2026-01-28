#include <VFF.h>

// 全局变量定义
int mission_num = 0;
float if_debug = 0;
float err_max = 0.2;
const float HOVER_DURATION = 10.0f; // 降落悬停时间（秒）
float hold_flag = false;

// 【可配置参数】
float target_x = 5.0f;     // 目标点x（相对起飞点，米）
float target_y = 0.0f;     // 目标点y（相对起飞点，米）
float target_z = ALTITUDE; // 目标点z高度（米）
float target_yaw = 0.0f;   // 目标偏航角（弧度）
float UAV_radius = 0.3f;   // 无人机等效半径（米）
float time_final = 70.0f;  // 超时时间（秒）

void print_param()
{
  std::cout << "=== 控制参数 ===" << std::endl;
  std::cout << "err_max: " << err_max << std::endl;
  std::cout << "ALTITUDE: " << ALTITUDE << std::endl;
  std::cout << "if_debug: " << if_debug << std::endl;
  if (if_debug == 1)
    cout << "自动offboard" << std::endl;
  else
    cout << "遥控器offboard" << std::endl;
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

  // 参数读取

  nh.param<float>("err_max", err_max, 0);
  nh.param<float>("if_debug", if_debug, 0);
  nh.param<float>("target_x", target_x, 5.0f);
  nh.param<float>("target_y", target_y, 0.0f);
  nh.param<float>("target_yaw", target_yaw, 0.0f);
  nh.param<float>("UAV_radius", UAV_radius, 0.3f);
  nh.param<float>("time_final", time_final, 70.0f);

  // nh.param<float>("ALPHA", ALPHA, 2.0f);
  nh.param<float>("MAX_SPEED", MAX_SPEED, 1.0f);
  // nh.param<float>("OBS_EPS", OBS_EPS, 0.1f);
  // nh.param<float>("KV", KV, 0.2f);
  // nh.param<float>("KN", KN, 0.05f);
  // nh.param<float>("W_goal", W_goal, 0.8f);
  // nh.param<float>("W_free", W_free, 0.2f);

  print_param();

  int choice = 0;
  std::cout << "1 to go on , else to quit" << std::endl;
  std::cin >> choice;
  if (choice != 1)
    return 0;
  ros::spinOnce();
  rate.sleep();

  // 等待连接到飞控
  while (ros::ok() && !current_state.connected)
  {
    ros::spinOnce();
    rate.sleep();
  }
  // 设置无人机的期望位置

  setpoint_raw.type_mask = /*1 + 2 + 4 + 8 + 16 + 32*/ +64 + 128 + 256 + 512 /*+ 1024 + 2048*/;
  setpoint_raw.coordinate_frame = 1;
  setpoint_raw.position.x = 0;
  setpoint_raw.position.y = 0;
  setpoint_raw.position.z = ALTITUDE;
  setpoint_raw.yaw = 0;

  // send a few setpoints before starting
  for (int i = 100; ros::ok() && i > 0; --i)
  {
    mavros_setpoint_pos_pub.publish(setpoint_raw);
    ros::spinOnce();
    rate.sleep();
  }
  std::cout << "ok" << std::endl;

  // 定义客户端变量，设置为offboard模式
  mavros_msgs::SetMode offb_set_mode;
  offb_set_mode.request.custom_mode = "OFFBOARD";

  // 定义客户端变量，请求无人机解锁
  mavros_msgs::CommandBool arm_cmd;
  arm_cmd.request.value = true;

  // 记录当前时间，并赋值给变量last_request
  ros::Time last_request = ros::Time::now();

  while (ros::ok())
  {
    if (current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(3.0)))
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
      if (!current_state.armed && (ros::Time::now() - last_request > ros::Duration(3.0)))
      {
        if (arming_client.call(arm_cmd) && arm_cmd.response.success)
        {
          ROS_INFO("Vehicle armed");
        }
        last_request = ros::Time::now();
      }
    }
    // 当无人机到达起飞点高度后，悬停3秒后进入任务模式，提高视觉效果
    if (fabs(local_pos.pose.pose.position.z - ALTITUDE) < 0.2)
    {
      if (ros::Time::now() - last_request > ros::Duration(1.0))
      {
        mission_num = 1;
        last_request = ros::Time::now();
        break;
      }
    }

    mission_pos_cruise(0, 0, ALTITUDE, 0, err_max);
    mavros_setpoint_pos_pub.publish(setpoint_raw);
    ros::spinOnce();
    rate.sleep();
  }

  while (ros::ok())
  {
    ROS_WARN("mission_num = %d", mission_num);

    switch (mission_num)
    {
    // mission1: 起飞
    case 1:
    {
      if (mission_pos_cruise(0, 0, ALTITUDE, 0, err_max))
      {
        mission_num = 2;
        last_request = ros::Time::now();
      }
      else if (ros::Time::now() - last_request >= ros::Duration(9.0))
      {
        mission_num = 2;
        last_request = ros::Time::now();
      }
      break;
    }

    // 世界系前进
    case 2:
    {
      // setpoint_raw.position.x = 1.f;
      // break;
      // 调用圆锥避障函数（每帧执行）
      bool is_finish = cone_avoidance_movement(target_x, target_y, target_z,
                                               target_yaw, UAV_radius,
                                               time_final, err_max);
      if (is_finish)
      {
        printf("圆锥避障任务完成，准备降落！\n");
        mission_num = 3; // 完成避障，切换到降落任务
        break;
      }
      // 修复15：仅在避障函数内发布速度指令，避免重复发布
      mavros_setpoint_pos_pub.publish(setpoint_raw);
      break;
    }

    case 3:
    {
      if (precision_land(err_max))
      {
        ROS_WARN("精确降落完成，任务结束！");
        mission_num = -1; // 任务结束
      }
      break;
    }
    }
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
