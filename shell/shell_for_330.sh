#!/bin/zsh

# Session 名称
SESSION="ros_session"

# ================= 配置路径 (实机版) =================
MAIN_WS=~/first_task_ws
LAND_WS=~/first_task_ws
# 实机通常不需要 SIM_WS 和 PX4_PATH (Gazebo 相关)

# 清理旧环境
tmux kill-session -t $SESSION 2>/dev/null
sleep 1

# ====================================================
# 窗口 0: 硬件驱动 (MAVROS + Camera)
# ====================================================
tmux new-session -d -s $SESSION -n "hardware_drivers"

# Pane 0.0: roscore
tmux send-keys -t $SESSION:0.0 'roscore' C-m
sleep 2

# Pane 0.1: 实机基础启动 (取代 sim.launch)
tmux split-window -h -t $SESSION:0
tmux send-keys -t $SESSION:0.1 "sleep 2; source ${MAIN_WS}/devel/setup.zsh; roslaunch tutorial_gazebo utils.launch" C-m

# ====================================================
# 窗口 1: PCL 感知 (实机检测)
# ====================================================
tmux new-window -t $SESSION:1 -n "pcl_perception"
# 实机中 obs.zsh 可能需要根据实机雷达话题调整
tmux send-keys -t $SESSION:1 "sleep 8; source ${MAIN_WS}/devel/setup.zsh; cd ${MAIN_WS}/src/pcl_detection/shell; zsh obs.zsh" C-m

# ====================================================
# 窗口 2: 任务控制与视觉 (Mission + YOLO)
# ====================================================
tmux new-window -t $SESSION:2 -n "mission_ctrl"

# Pane 2.0: 话题监控
tmux send-keys -t $SESSION:2.0 "sleep 5; rostopic echo /mavros/local_position/pose" C-m

# Pane 2.1: A* 主控节点
tmux split-window -v -t $SESSION:2.0
tmux send-keys -t $SESSION:2.1 "sleep 12; source ${MAIN_WS}/devel/setup.zsh; roslaunch astar astar.launch" C-m

# Pane 2.2: YOLO 圆环检测 (实机建议确认是否开启 TensorRT 加速)
tmux split-window -h -t $SESSION:2.1
tmux send-keys -t $SESSION:2.2 "sleep 10; source ${MAIN_WS}/devel/setup.zsh; rosrun astar ring_detector.py" C-m

# ====================================================
# 窗口 3: 视觉起降识别 (Scan Land)
# ====================================================
tmux new-window -t $SESSION:3 -n "scan_land"

# 实机环境建议先启动感知 Python 节点
tmux send-keys -t $SESSION:3.0 "sleep 8; source ${LAND_WS}/devel/setup.zsh; roslaunch scan_land scan_land_py.launch" C-m

# 启动主逻辑
tmux split-window -v -t $SESSION:3.0
tmux send-keys -t $SESSION:3.1 "sleep 15; source ${LAND_WS}/devel/setup.zsh; roslaunch scan_land scan_land.launch" C-m

# ====================================================
# 窗口 4: 相机驱动
# ====================================================
tmux new-window -t $SESSION:4 -n "camera_driver"

# 实机环境建议先启动感知 Python 节点
tmux send-keys -t $SESSION:4.0 "sleep 8; source ${LAND_WS}/devel/setup.zsh; roslaunch scan_land simple_camera_driver.launch" C-m
tmux split-window -v -t $SESSION:4.0
tmux send-keys -t $SESSION:4.1 "sleep 8; source ${LAND_WS}/devel/setup.zsh; roslaunch scan_land simple_camera_front_driver.launch" C-m


# ====================================================
# 收尾
# ====================================================
tmux select-layout -t $SESSION:2 tiled
tmux select-window -t $SESSION:2
tmux select-pane -t $SESSION:2.1
tmux attach-session -t $SESSION
