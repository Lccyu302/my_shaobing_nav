#!/bin/zsh
sleep 6
# 进入工作空间目录
cd /home/xinzhu/Desktop/sentry_ws 

# 加载 ROS2 基础环境（假设你的默认 shell 是 zsh）
source /opt/ros/humble/setup.bash
# 加载工作空间的环境（使用 .zsh 文件）
source install/setup.zsh

# 打开新终端并执行 launch（保留终端不关闭）
gnome-terminal -- zsh -c "ros2 launch sentry_bringup bringup_without_relocalization.py; exec zsh"
