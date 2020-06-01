#! /bin/bash
rm -rf /root/cps_challenge_2020
wget -r -npH https://download.openuas.us/cps_challenge_2020/ -P ./
rsync -aP cps_challenge_2020/ /root/catkin_ws/src/cps_challenge_2020/
rm -rf /root/cps_challenge_2020
