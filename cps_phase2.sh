#! /bin/bash
echo "export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/root/catkin_ws/src/cps_challenge_2020/models" >> ~/.bashrc
wget https://download.openuas.us/cps_challenge_2020/launch/phase-1.launch -O /root/catkin_ws/src/cps_challenge_2020/launch/phase-1.launch
wget https://download.openuas.us/cps_challenge_2020/launch/phase-2.launch -O /root/catkin_ws/src/cps_challenge_2020/launch/phase-2.launch
wget https://download.openuas.us/cps_challenge_2020/worlds/phase-1.world -O /root/catkin_ws/src/cps_challenge_2020/worlds/phase-1.world
wget https://download.openuas.us/cps_challenge_2020/worlds/phase-2.world -O /root/catkin_ws/src/cps_challenge_2020/worlds/phase-2.world
wget https://download.openuas.us/cps_challenge_2020/launch/rover_world.launch -O /root/catkin_ws/src/cps_challenge_2020/launch/rover_world.launch

/root/catkin_ws/src/cps_challenge_2020/launch/rover_world.launch
wget https://download.openuas.us/cps_challenge_2020/models/iris/iris.sdf -O /root/catkin_ws/src/cps_challenge_2020/models/iris/iris.sdf

wget --recursive --no-parent -nH https://download.openuas.us/cps_challenge_2020/models/cps_depth_camera/ -P /root/catkin_ws/src/
wget --recursive --no-parent -nH https://download.openuas.us/cps_challenge_2020/models/cps_rover/ -P /root/catkin_ws/src/
wget --recursive --no-parent -nH https://download.openuas.us/cps_challenge_2020/models/cps_depth_camera/ -P /root/catkin_ws/src/
cp /root/catkin_ws/src/cps_challenge_2020/models/iris/iris.sdf /root/src/Firmware/Tools/sitl_gazebo/models/iris/iris.sdf
apt-get -y install xmlstarlet

