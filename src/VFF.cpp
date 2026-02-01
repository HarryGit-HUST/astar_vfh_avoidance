#include <VFH.h>

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

// VFF算法参数（全部从yaml读取）
float safe_margin = 0.5f;          // 安全裕度（米）
float repulsive_gain = 2.0f;       // 排斥力增益系数（VFF专用）
float MAX_SPEED = 1.0f;            // 最大前进速度（米/秒）
float MIN_SAFE_DISTANCE = 0.2f;    // 力场计算最小距离（防除零）
float MAX_REPULSIVE_FORCE = 10.0f; // 排斥力上限（VFF专用）

// ========== 新增：VFH+专用参数 ==========
float HISTOGRAM_THRESHOLD = 40.0f; // 直方图拥堵阈值（0~100）
float SMOOTHING_RADIUS = 2.0f;     // 直方图平滑半径（扇区数）
bool USE_VFH_PLUS = true;          // true=使用VFH+，false=使用VFF
void print_param()
{
  std::cout << "=== 控制参数 ===" << std::endl;
  std::cout << "err_max: " << err_max << std::endl;
  std::cout << "ALTITUDE: " << ALTITUDE << std::endl;
  std::cout << "if_debug: " << if_debug << std::endl;
  std::cout << "USE_VFH_PLUS: " << (USE_VFH_PLUS ? "true (VFH+)" : "false (VFF)") << std::endl;

  if (if_debug == 1)
    cout << "自动offboard + 详细避障调试日志" << std::endl;
  else
    cout << "遥控器offboard + 简洁日志" << std::endl;

  // 打印避障参数
  std::cout << "\n=== 避障参数（yaml配置） ===" << std::endl;
  std::cout << "UAV_radius: " << UAV_radius << " m" << std::endl;
  std::cout << "safe_margin: " << safe_margin << " m" << std::endl;
  std::cout << "MAX_SPEED: " << MAX_SPEED << " m/s" << std::endl;
  std::cout << "MIN_SAFE_DISTANCE: " << MIN_SAFE_DISTANCE << " m" << std::endl;

  if (USE_VFH_PLUS)
  {
    std::cout << "HISTOGRAM_THRESHOLD: " << HISTOGRAM_THRESHOLD << " (拥堵阈值)" << std::endl;
    std::cout << "SMOOTHING_RADIUS: " << SMOOTHING_RADIUS << " 扇区" << std::endl;
  }
  else
  {
    std::cout << "repulsive_gain: " << repulsive_gain << " (VFF排斥增益)" << std::endl;
    std::cout << "MAX_REPULSIVE_FORCE: " << MAX_REPULSIVE_FORCE << " (VFF力上限)" << std::endl;
  }
}

int main(int argc, char **argv)
{
  // 防止中文输出乱码
  setlocale(LC_ALL, "");

  // 初始化ROS节点
  ros::init(argc, argv, "VFH");
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
  nh.param<bool>("USE_VFH_PLUS", USE_VFH_PLUS, true); // 新增开关

  // 通用避障参数
  nh.param<float>("safe_margin", safe_margin, 0.5f);
  nh.param<float>("MAX_SPEED", MAX_SPEED, 1.0f);
  nh.param<float>("MIN_SAFE_DISTANCE", MIN_SAFE_DISTANCE, 0.2f);

  // VFF专用参数（仅当USE_VFH_PLUS=false时使用）
  nh.param<float>("repulsive_gain", repulsive_gain, 2.0f);
  nh.param<float>("MAX_REPULSIVE_FORCE", MAX_REPULSIVE_FORCE, 10.0f);

  // VFH+专用参数（仅当USE_VFH_PLUS=true时使用）
  nh.param<float>("HISTOGRAM_THRESHOLD", HISTOGRAM_THRESHOLD, 40.0f);
  nh.param<float>("SMOOTHING_RADIUS", SMOOTHING_RADIUS, 2.0f);
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

      // 世界系前进（智能避障：VFH+ 或 VFF）
    case 2:
    {
      bool reached = false;

      if (USE_VFH_PLUS)
      {
        // ========== VFH+调用（推荐） ==========
        reached = vfh_avoidance(
            target_x,            // 目标X（相对）
            target_y,            // 目标Y（相对）
            target_yaw,          // 目标航向
            UAV_radius,          // 无人机半径
            safe_margin,         // 安全裕度
            MAX_SPEED,           // 最大速度
            MIN_SAFE_DISTANCE,   // 最小安全距离
            HISTOGRAM_THRESHOLD, // 拥堵阈值
            SMOOTHING_RADIUS     // 平滑半径
        );
      }
      else
      {
        // ========== VFF调用（兼容旧版） ==========
        reached = vff_avoidance(
            target_x,
            target_y,
            target_yaw,
            UAV_radius,
            safe_margin,
            repulsive_gain,
            MAX_SPEED,
            MIN_SAFE_DISTANCE,
            MAX_REPULSIVE_FORCE);
      }

      // 任务完成判断
      ros::Duration elapsed = ros::Time::now() - last_request;
      if (reached || elapsed.toSec() > time_final)
      {
        if (reached)
        {
          ROS_WARN("[避障] ✓ 成功抵达目标点(%.2f, %.2f)!", target_x, target_y);
        }
        else
        {
          ROS_WARN("[避障] ✗ 超时保护触发(%.1fs)，强制结束", elapsed.toSec());
        }
        mission_num = 3;
        last_request = ros::Time::now();
      }
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