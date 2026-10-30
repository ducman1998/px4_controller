## Installation
### Install MAVROS in ROS1
sudo apt update 
sudo apt install -y ros-noetic-mavros ros-noetic-mavros-extras ros-noetic-mavros-msgs
sudo apt install -y geographiclib-tools libgeographic-dev

Then run this script to download geographiclib:
/opt/ros/noetic/lib/mavros/install_geographiclib_datasets.sh


## Run
Start QGroundControl app

Run Gazebo simulation for PX4 drone:
cd ~/workspace/PX4-Vision/SITL/PX4-Autopilot
make px4_sitl gazebo

Start Mavros bridge
roslaunch mavros px4.launch fcu_url:="udp://:14540@127.0.0.1:14557"

Run PX4 controller
roslaunch xxx
rosrun px4_controller simple_mission.py





