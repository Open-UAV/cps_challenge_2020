#! /bin/bash
git clone https://github.com/Open-UAV/cps_challenge_2020
rsync -aP cps_challenge_2020 /root/catkin_ws/src/cps_challenge_2020/
wget https://download.openuas.us/cps_challenge_2020/models/iris/iris.sdf -O /root/catkin_ws/src/cps_challenge_2020/models/iris/iris.sdf
cp /root/catkin_ws/src/cps_challenge_2020/models/iris/iris.sdf /root/src/Firmware/Tools/sitl_gazebo/models/iris/iris.sdf
apt-get -y install xmlstarlet

