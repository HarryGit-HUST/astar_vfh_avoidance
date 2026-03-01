#!/bin/bash

# Session 名称
SESSION="ros_session"

# ================= 配置路径 (根据你的报错截图修正) =================
# 1. 算法工作空间
MAIN_WS=~/first_task_ws
# 2. 仿真工作空间
SIM_WS=~/catkin_ws
# 3. PX4 固件路径 (关键修复点！)
PX4_PATH=/home/jetson/Libraries/PX4-Autopilot

# 清理旧环境
tmux kill-session -t $SESSION 2>/dev/null
sleep 1

# ====================================================
# 窗口 0: 基础仿真 (Sim + Core)
# ====================================================
tmux new-session -d -s $SESSION -n "sim_core"

# Pane 0.0: roscore
tmux send-keys -t $SESSION:0.0 'roscore' C-m
sleep 2

# Pane 0.1 (右): Gazebo 仿真 (修复环境配置)
tmux split-window -h -t $SESSION:0
# 解释：这里不仅要 source sim_ws，还要 source PX4 的环境脚本，并导出 ROS_PACKAGE_PATH
tmux send-keys -t $SESSION:0.1 "sleep 3; \
source ${SIM_WS}/devel/setup.bash; \
source ${PX4_PATH}/Tools/setup_gazebo.bash ${PX4_PATH} ${PX4_PATH}/build/px4_sitl_default; \
export ROS_PACKAGE_PATH=\$ROS_PACKAGE_PATH:${PX4_PATH}:${PX4_PATH}/Tools/sitl_gazebo; \
roslaunch tutorial_gazebo sim.launch" C-m

# ====================================================
# 窗口 1: PCL 感知 (单独监视)
# ====================================================
tmux new-window -t $SESSION:1 -n "pcl_perception"

# PCL 启动 (Source 主算法工作空间)
# 延时 10s 等待 Gazebo 加载
tmux send-keys -t $SESSION:1 "sleep 10; source ${MAIN_WS}/devel/setup.bash; cd ${MAIN_WS}/src/pcl_detection/shell; bash obs.bash" C-m

# ====================================================
# 窗口 2: 任务控制与视觉 (Mission + YOLO)
# ====================================================
tmux new-window -t $SESSION:2 -n "mission_ctrl"

# Pane 2.0 (上): 话题监控
tmux send-keys -t $SESSION:2.0 "sleep 5; rostopic echo /mavros/local_position/pose" C-m

# Pane 2.1 (左下): A* 主控节点 (Source 主算法工作空间)
tmux split-window -v -t $SESSION:2.0
# 启动 astar.launch，延时 15s 确保 PCL 已经准备好
tmux send-keys -t $SESSION:2.1 "sleep 15; source ${MAIN_WS}/devel/setup.bash; roslaunch astar astar.launch" C-m

# Pane 2.2 (右下): YOLO 视觉节点 (Source 主算法工作空间)
tmux split-window -h -t $SESSION:2.1
# 启动 ring_detector.py
tmux send-keys -t $SESSION:2.2 "sleep 12; source ${MAIN_WS}/devel/setup.bash; rosrun astar ring_detector.py" C-m

# 布局调整
tmux select-layout -t $SESSION:2 tiled

# ====================================================
# 收尾
# ====================================================
# 默认聚焦到主控窗口 (Window 2) 的 A* 终端，方便输入 "1"
tmux select-window -t $SESSION:2
tmux select-pane -t $SESSION:2.1
tmux attach-session -t $SESSION