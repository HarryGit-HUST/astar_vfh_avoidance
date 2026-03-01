#ifndef RING_CROSSING_H
#define RING_CROSSING_H

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <mavros_msgs/PositionTarget.h>
#include <nav_msgs/Odometry.h>
#include <cmath>

class RingCrossing
{
public:
    RingCrossing();

    // 视觉回调：接收 Python 发来的 /ring_center
    void vision_cb(const geometry_msgs::Point::ConstPtr &msg);

    // 核心计算函数：根据视觉和里程计，计算下一步的控制指令
    // 返回 true 表示判定已穿过
    bool compute_cmd(const nav_msgs::Odometry &local_pos,
                     double current_yaw,
                     mavros_msgs::PositionTarget &setpoint);

    // 重置状态（每次开始穿门任务前调用）
    void reset();

private:
    // YOLO 数据
    float yolo_u, yolo_v, yolo_w;
    ros::Time last_yolo_time;

    // 记忆坐标 (EMA滤波用)
    float estimated_gate_y;
    float estimated_gate_z;
    bool has_gate_estimate;

    // 穿门状态位
    bool is_crossing_blind; // 是否进入盲穿冲刺阶段
};

#endif