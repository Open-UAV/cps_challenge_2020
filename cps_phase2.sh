#! /bin/bash
rm -rf /root/cps_challenge_2020
git clone https://github.com/Open-UAV/cps_challenge_2020
rsync -aP cps_challenge_2020/ /root/catkin_ws/src/cps_challenge_2020/
apt-get -y install xmlstarlet
