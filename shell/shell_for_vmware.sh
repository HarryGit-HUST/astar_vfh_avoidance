#!/bin/bash

# Session 名称
SESSION="ros_session"

# 清理旧环境
tmux kill-session -t $SESSION 2>/dev/null
sleep 1

# ====================================================
# 窗口 0: 核心仿真与感知 (Sim + PCL)
# ====================================================
tmux new-session -d -s $SESSION -n "sim_core"

# Pane 0.0: roscore
tmux send-keys -t $SESSION:0.0 'roscore' C-m
sleep 2

# Pane 0.1 (右): Gazebo 仿真
tmux split-window -h -t $SESSION:0
tmux send-keys -t $SESSION:0.1 'sleep 3; roslaunch tutorial_gazebo sim.launch' C-m

# Pane 0.2 (下): PCL 障碍物检测
tmux split-window -v -t $SESSION:0.0
tmux send-keys -t $SESSION:0.2 'sleep 8; roslaunch pcl_detection object_detector.launch' C-m

# 布局调整
tmux select-layout -t $SESSION:0 tiled

# ====================================================
# 窗口 1: 任务控制与视觉 (Mission + YOLO)
# ====================================================
tmux new-window -t $SESSION:1 -n "mission_ctrl"

# Pane 1.0 (上): 话题监控
tmux send-keys -t $SESSION:1.0 'sleep 5; rostopic echo /mavros/local_position/pose' C-m

# Pane 1.1 (左下): A* 主控节点
tmux split-window -v -t $SESSION:1.0
# 注意：这里假设你的 astar 包名就是 "astar"，且 launch 文件正确
tmux send-keys -t $SESSION:1.1 'sleep 10; source ~/first_task_ws/devel/setup.bash; roslaunch astar astar.launch' C-m

# Pane 1.2 (右下): YOLO 视觉节点 (Python)
tmux split-window -h -t $SESSION:1.1
# 注意：确保 ring_detector.py 有执行权限 (chmod +x)
tmux send-keys -t $SESSION:1.2 'sleep 9; source ~/first_task_ws/devel/setup.bash; rosrun astar ring_detector.py' C-m

# 布局调整
tmux select-layout -t $SESSION:1 tiled

# ====================================================
# 收尾
# ====================================================
# 默认聚焦到主控窗口，方便输入 "1" 开始任务
tmux select-window -t $SESSION:1
tmux select-pane -t $SESSION:1.1
tmux attach-session -t $SESSION