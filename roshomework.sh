#!/bin/bash
clear
echo "机器人201 张明琪 王若涵 大作业"

echo "—————————————————————————————————————————————"
echo "是否安装Turtlebot3库"
echo "—————————————————————————————————————————————"
read -p "是否安装Turtlebot3库（Y/N）" value

if [ "$value" == "Y"  ] || [  "$value" == "y" ]; then
echo "—————————————————————————————————————————————"
echo "安装Turtlebot3库"
echo "—————————————————————————————————————————————"
   sudo apt-get install ros-melodic-joy ros-melodic-teleop-twist-joy ros-melodic-teleop-twist-keyboard ros-melodic-laser-proc ros-melodic-rgbd-launch ros-melodic-depthimage-to-laserscan ros-melodic-rosserial-arduino ros-melodic-rosserial-python ros-melodic-rosserial-server ros-melodic-rosserial-client ros-melodic-rosserial-msgs ros-melodic-amcl ros-melodic-map-server ros-melodic-move-base ros-melodic-urdf ros-melodic-xacro  ros-melodic-compressed-image-transport ros-melodic-rqt-image-view ros-melodic-gmapping ros-melodic-navigation  ros-melodic-interactive-markers rviz
   mkdir -p ~/turtlebot3_ws/src/
    cd ~/turtlebot3_ws/src/
    git clone https://gitee.com/MingQi_Ya/turtlebot3_simulations.git
    git clone https://gitee.com/MingQi_Ya/turtlebot3_msgs.git
    git clone https://gitee.com/MingQi_Ya/turtlebot3.git

    cd ~/turtlebot3_ws
    catkin_make

    echo "source ~/turtlebot3_ws/devel/setup.bash " >> ~/.bashrc
    echo " export TURTLEBOT3_MODEL=waffle " >> ~/.bashrc

    cd ~
    source ./.bashrc 
echo "—————————————————————————————————————————————"
echo "安装Turtlebot3库--成功"
echo "—————————————————————————————————————————————"
fi

if [ "$value" == "N"  ] || [  "$value" == "n" ]; then
    echo "—————————————————————————————————————————————"
    echo "跳过安装Turtlebot3库"
    echo "—————————————————————————————————————————————"
fi

read -p "是否安装gazebo模型库（Y/N）" value1

if [ "$value1" == "Y"  ] || [  "$value1" == "y" ]; then
echo "—————————————————————————————————————————————"
echo "安装gazebo模型库"
echo "—————————————————————————————————————————————"
# 进入.gazebo文件夹，用于存放Gazebo模型和相关配置文件
   cd ~/.gazebo/
   # 安装Git工具
   sudo apt install git
   # 从https://gitee.com/dva7777/gazebo_models.git克隆Gazebo模型
   git clone https://gitee.com/dva7777/gazebo_models.git
   # 将克隆的gazebo_models文件夹重命名为models，并放到.gazebo文件夹下
   mv ~/.gazebo/models/gazebo_models/* ~/.gazebo/models/
   sudo rm -r ~/models/gazebo_models/

   cd ~
   source ./.bashrc 
echo "—————————————————————————————————————————————"
echo "安装gazebo模型库--成功"
echo "—————————————————————————————————————————————"
fi
if [ "$value1" == "N"  ] || [  "$value1" == "n" ]; then
    echo "—————————————————————————————————————————————"
    echo "跳过安装gazebo模型库"
    echo "—————————————————————————————————————————————"
fi

read -p "是否安装ROS大作业-张明琪 王若涵（Y/N）" value2

if [ "$value2" == "Y"  ] || [  "$value2" == "y" ]; then
echo "—————————————————————————————————————————————"
echo "安装功能包"
echo "—————————————————————————————————————————————"
  sudo apt-get install ros-melodic-gmapping
  sudo apt-get install ros-melodic-dwa-local-planner
   cd ~
   source ./.bashrc 
echo "—————————————————————————————————————————————"
echo "安装功能包--成功"
echo "—————————————————————————————————————————————"


echo "—————————————————————————————————————————————"
echo "创建工作空间"
echo "—————————————————————————————————————————————"
   #创建文件夹
   mkdir -p ~/roshomework/src
   cd ~/roshomework/src
   # ROS的工作空间初始化命令
   catkin_init_workspace
   cd ..
   # 编译整个工作空间
   catkin_make
   #配置文件中加入环境变量
   echo "source ~/roshomework/devel/setup.bash" >> ~/.bashrc
    cd ~
    source ./.bashrc 
echo "—————————————————————————————————————————————"
echo "创建工作空间--成功"
echo "—————————————————————————————————————————————"


echo "—————————————————————————————————————————————"
echo "下载源文件"
echo "—————————————————————————————————————————————"
   cd ~/roshomework/src
   #git下载文件
    git clone https://gitee.com/MingQi_Ya/ROS-work.git
    #移动文件
    mv ~/roshomework/src/ROS-work/* ~/roshomework/src
    sudo rm -r ~/roshomework/src/ROS-work
    #更新环境变量
    cd ~
    source ./.bashrc 
echo "—————————————————————————————————————————————"
echo "下载源文件--成功"
echo "—————————————————————————————————————————————"
fi
if [ "$value2" == "N"  ] || [  "$value2" == "n" ]; then
    echo "—————————————————————————————————————————————"
    echo "跳过安装"
    echo "—————————————————————————————————————————————"
fi

echo "—————————————————————————————————————————————"
echo "打开新的终端输入以下"
echo "运行slam导航的launch"
echo "roslaunch ros-w turtlebot3_guidance.launch"
echo "—————————————————————————————————————————————"

echo "—————————————————————————————————————————————"
echo "安装成功，再见"
echo "—————————————————————————————————————————————"