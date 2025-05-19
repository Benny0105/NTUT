#!/bin/bash

# 定義 PX4 根目錄
PX4_ROOT=~/Desktop/PX4-Autopilot
WORKSPACE=~/Benny/catkin_ws
WORLD_PATH=~/.gazebo/worlds/custom_world.world  # 你的自訂世界文件

# 啟動 ROS Master
echo "Starting ROS Master..."
gnome-terminal -- bash -c "source /opt/ros/noetic/setup.bash && roscore; exec bash" &
sleep 5

# 設置 PX4 環境變數
source $PX4_ROOT/Tools/simulation/gazebo-classic/setup_gazebo.bash $PX4_ROOT $PX4_ROOT/build/px4_sitl_default
export ROS_PACKAGE_PATH=$PX4_ROOT:$PX4_ROOT/Tools/simulation/gazebo-classic:$ROS_PACKAGE_PATH
export GAZEBO_PLUGIN_PATH=$PX4_ROOT/build/px4_sitl_default/build_gazebo-classic:$GAZEBO_PLUGIN_PATH
export GAZEBO_MODEL_PATH=$PX4_ROOT/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models:$GAZEBO_MODEL_PATH
export PX4_HOME_LAT=23.457822 export PX4_HOME_LON=120.276008 export PX4_HOME_ALT=0

# 確保沒有多個 mavros 運行
echo "Checking for existing MAVROS instances..."
if rosnode list | grep -q "/mavros"; then
    echo "MAVROS is already running. Killing existing instance..."
    rosnode kill /mavros
    sleep 2
fi

# 啟動 PX4 SITL 與 Gazebo
echo "Starting PX4 SITL with Gazebo and MAVROS (using your ekf/mavros_posix_sitl.launch)..."
gnome-terminal -- bash -c "\
  source /opt/ros/noetic/setup.bash && \
  source $PX4_ROOT/Tools/simulation/gazebo-classic/setup_gazebo.bash \
         $PX4_ROOT $PX4_ROOT/build/px4_sitl_default && \
  source $WORKSPACE/devel/setup.bash && \
  export ROS_PACKAGE_PATH=$PX4_ROOT:$PX4_ROOT/Tools/simulation/gazebo-classic:$WORKSPACE/src:$ROS_PACKAGE_PATH && \
  roslaunch ekf mavros_posix_sitl.launch; \
  exec bash" &
  
# 增加延遲，確保 MAVROS 完全初始化
echo "Waiting for MAVROS to initialize (30 seconds)..."
sleep 15

# 啟動里程計轉換節點
echo "Starting Odometry Republisher..."
gnome-terminal -- bash -c "source /opt/ros/noetic/setup.bash && source $WORKSPACE/devel/setup.bash && rosrun ekf republish_odom.py; exec bash" &
sleep 3
echo "Odometry republisher started!"

# 檢查 GPS 話題是否可用
echo "Checking if /mavros/global_position/raw/fix topic is available..."
if rostopic list | grep -q "/mavros/global_position/raw/fix"; then
    echo "Raw GPS topic found! Starting GPS bridge..."
    # 啟動 GPS 橋接節點
    gnome-terminal -- bash -c "source /opt/ros/noetic/setup.bash && source $WORKSPACE/devel/setup.bash && rosrun ekf gps_bridge.py; exec bash" &
    sleep 3
    echo "GPS bridge started!"
else
    echo "Warning: Raw GPS topic not found! GPS data may not be available."
fi

# 啟動 GPS 控制節點 (新增部分)
echo "Starting GPS Control Node..."
gnome-terminal -- bash -c "source /opt/ros/noetic/setup.bash && source $WORKSPACE/devel/setup.bash && rosrun ekf gps_control_node.py _mavros_gps_topic:=/mavros/global_position/raw/fix _mavros_gps_vel_topic:=/mavros/global_position/raw/gps_vel _mavros_gps_sat_topic:=/mavros/global_position/raw/satellites _ekf_gps_topic:=/filtered/gps_data _ekf_gps_vel_topic:=/filtered/gps_vel_data _ekf_gps_sat_topic:=/filtered/gps_sat_data; exec bash" &
sleep 2
echo "GPS Control Node started!"

# 提示用戶啟動 EKF
echo "PX4 SITL, GPS bridge, and GPS control node have been started."
echo "To start the EKF, open a new terminal and run: roslaunch ekf px4_ekf_sim.launch"
echo "注意: 啟動 EKF 時，請確保使用修改後的話題映射，將 EKF 中的 GPS 話題指向 /filtered/* 話題"
sleep 3

# 啟動無人機飛行 + 拍照腳本
echo "Starting Python drone flight and image capture script..."
gnome-terminal -- bash -c "source /opt/ros/noetic/setup.bash && source $WORKSPACE/devel/setup.bash && rosrun ekf control3.py; exec bash"

echo "Done"

