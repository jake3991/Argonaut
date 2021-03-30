export ROS_HOSTNAME=192.168.2.10
export ROS_MASTER_URI=http://192.168.2.10:11311

sudo chmod 777 /dev/ttyUSB0
sudo chmod 777 /dev/ttyUSB1


source Aquaman/devel/setup.bash

roslaunch bluerov_launch bluerov_launch.launch 



