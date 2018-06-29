version_pub 
1.modify new version : robot_state_keeper robot_state_keeper.h  VERSION_INFO
                       version_pub.sh  pubname
2.add new version info : robot_state_keeper/info/version_release.txt
2.cd src/robot_state_keeper/shell/
3. ./version_pub.sh

upgrade.h
1.cp navigation.tar.gz to /home/robot
2.cd ~/catkin_ws/install/share/robot_state_keeper/shell
3. ./upgrade.sh
