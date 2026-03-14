#!/bin/bash

# Session 名称
SESSION="ros_session"

# ================= 配置路径 =================
MAIN_WS=~/first_task_ws
SIM_WS=~/catkin_ws
LAND_WS=~/first_task_ws # 新增：视觉起降工作空间
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

# Pane 0.1: Gazebo 仿真
tmux split-window -h -t $SESSION:0
tmux send-keys -t $SESSION:0.1 "sleep 3; \
source ${SIM_WS}/devel/setup.bash; \
source ${PX4_PATH}/Tools/setup_gazebo.bash ${PX4_PATH} ${PX4_PATH}/build/px4_sitl_default; \
export ROS_PACKAGE_PATH=\$ROS_PACKAGE_PATH:${PX4_PATH}:${PX4_PATH}/Tools/sitl_gazebo; \
roslaunch tutorial_gazebo sim.launch" C-m

# ====================================================
# 窗口 1: PCL 感知
# ====================================================
tmux new-window -t $SESSION:1 -n "pcl_perception"
tmux send-keys -t $SESSION:1 "sleep 10; source ${MAIN_WS}/devel/setup.bash; cd ${MAIN_WS}/src/pcl_detection2/shell; bash pcl_detection.sh" C-m

# ====================================================
# 窗口 2: 任务控制与视觉 (Mission + YOLO)
# ====================================================
tmux new-window -t $SESSION:2 -n "mission_ctrl"

# Pane 2.0: 话题监控
tmux send-keys -t $SESSION:2.0 "sleep 5; rostopic echo /mavros/local_position/pose" C-m

# Pane 2.1: A* 主控节点
tmux split-window -v -t $SESSION:2.0
tmux send-keys -t $SESSION:2.1 "sleep 15; source ${MAIN_WS}/devel/setup.bash; roslaunch astar astar.launch" C-m

# Pane 2.2: YOLO 圆环检测
tmux split-window -h -t $SESSION:2.1
tmux send-keys -t $SESSION:2.2 "sleep 12; source ${MAIN_WS}/devel/setup.bash; rosrun astar ring_detector.py" C-m

# ====================================================
# 窗口 3: 视觉起降识别 (Scan Land) - 新增
# ====================================================
tmux new-window -t $SESSION:3 -n "scan_land"

# Pane 3.0: 视觉起降 Python 节点 (根据你的指令先启动)
tmux send-keys -t $SESSION:3.0 "sleep 10; source ${LAND_WS}/devel/setup.bash; roslaunch scan_land scan_land_py.launch" C-m

# Pane 3.1: 视觉起降主节点 (后启动，有 7s 间隔)
tmux split-window -v -t $SESSION:3.0
tmux send-keys -t $SESSION:3.1 "sleep 17; source ${LAND_WS}/devel/setup.bash; roslaunch scan_land scan_land.launch" C-m

# ====================================================
# 收尾
# ====================================================
tmux select-layout -t $SESSION:2 tiled
tmux select-window -t $SESSION:2
tmux select-pane -t $SESSION:2.1
tmux attach-session -t $SESSION
