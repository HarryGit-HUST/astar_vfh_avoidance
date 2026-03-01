#include "ring_crossing.h"

RingCrossing::RingCrossing()
{
    reset();
}

void RingCrossing::reset()
{
    yolo_u = 320.0; // 默认图像中心
    yolo_v = 240.0;
    yolo_w = 0.0;
    last_yolo_time = ros::Time(0);
    has_gate_estimate = false;
    estimated_gate_y = 0.0;
    estimated_gate_z = 0.0;
    is_crossing_blind = false;
}

void RingCrossing::vision_cb(const geometry_msgs::Point::ConstPtr &msg)
{
    yolo_u = msg->x;
    yolo_v = msg->y;
    yolo_w = msg->z; // 宽度
    last_yolo_time = ros::Time::now();
}

bool RingCrossing::compute_cmd(const nav_msgs::Odometry &local_pos,
                               double current_yaw,
                               mavros_msgs::PositionTarget &setpoint)
{
    // === 1. 相机内参 (参考队友的数据) ===
    const float focal_length = 343.5;
    const float img_cx = 320.0;
    const float img_cy = 240.0;
    const float W_real = 0.9; // 门的真实宽度 (米)

    float safe_w = (yolo_w > 10.0) ? yolo_w : 10.0;

    // 计算相对于相机的距离
    float dist_x = (focal_length * W_real) / safe_w;
    float dist_y = -((yolo_u - img_cx) * dist_x) / focal_length;
    float dist_z = -((yolo_v - img_cy) * dist_x) / focal_length;

    // === 2. 动态记忆更新 (EMA) ===
    bool is_vision_valid = (ros::Time::now() - last_yolo_time).toSec() < 0.5;

    // 只有在距离适中且视野清晰时，更新世界坐标记忆
    if (is_vision_valid && dist_x > 1.0 && dist_x < 6.0)
    {
        // 坐标变换：相机系 -> 世界系 (简化版，假设相机朝前)
        float current_gate_y = local_pos.pose.pose.position.y + dist_x * sin(current_yaw) + dist_y * cos(current_yaw);
        float current_gate_z = local_pos.pose.pose.position.z + dist_z;

        if (!has_gate_estimate)
        {
            estimated_gate_y = current_gate_y;
            estimated_gate_z = current_gate_z;
            has_gate_estimate = true;
        }
        else
        {
            // EMA 滤波: 0.1 新数据 + 0.9 记忆
            estimated_gate_y = 0.1 * current_gate_y + 0.9 * estimated_gate_y;
            estimated_gate_z = 0.1 * current_gate_z + 0.9 * estimated_gate_z;
        }
    }

    // === 3. 盲穿判定 ===
    // 距离非常近(<1.5m)，或者框非常大(贴脸了)，或者丢失视野但有记忆 -> 进入冲刺模式
    if (dist_x < 1.5 || safe_w > 400.0 || (has_gate_estimate && !is_vision_valid))
    {
        is_crossing_blind = true;
    }

    // === 4. 生成控制指令 ===
    // 忽略速度控制，使用位置控制 (Position + Yaw)
    setpoint.type_mask = 0b100111111000;
    setpoint.coordinate_frame = 1; // Local NED

    if (is_crossing_blind)
    {
        // 冲刺模式：向机头方向冲，Y/Z轴锁死在记忆坐标
        float step_forward = 0.8; // 冲刺步长
        setpoint.position.x = local_pos.pose.pose.position.x + step_forward * cos(current_yaw);

        if (has_gate_estimate)
        {
            setpoint.position.y = estimated_gate_y;
            setpoint.position.z = estimated_gate_z;
        }
        else
        {
            // 如果没记忆，保持当前状态直飞
            setpoint.position.y = local_pos.pose.pose.position.y + step_forward * sin(current_yaw);
            setpoint.position.z = local_pos.pose.pose.position.z;
        }

        // 退出条件：这里只是单纯的控制，外部状态机需要判断是否越过了门的位置
        // 这里返回 false，让外部判断 X 坐标
    }
    else
    {
        // 视觉伺服模式：慢慢靠近，对准中心
        float Kp_y = 0.5;
        float Kp_z = 0.8;
        float step_forward = 0.3; // 慢速接近

        float align_error = sqrt(dist_y * dist_y + dist_z * dist_z);
        // 对不准就减速
        if (align_error > 0.2)
            step_forward = 0.1;

        // 计算世界坐标下的偏移
        float delta_X = step_forward * cos(current_yaw) - (dist_y * Kp_y) * sin(current_yaw);
        float delta_Y = step_forward * sin(current_yaw) + (dist_y * Kp_y) * cos(current_yaw);

        setpoint.position.x = local_pos.pose.pose.position.x + delta_X;
        setpoint.position.y = local_pos.pose.pose.position.y + delta_Y;
        setpoint.position.z = local_pos.pose.pose.position.z + (dist_z * Kp_z);
    }

    setpoint.yaw = current_yaw; // 保持当前偏航，或者你可以加逻辑让它对准门

    return is_crossing_blind; // 返回是否处于盲穿状态供调试
}