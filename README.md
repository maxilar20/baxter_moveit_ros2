TERMINAL 1
sudo docker run -it --net=host ros2_baxter_bridge
source opt/ros/noetic/setup.bash
source ros_ws/devel/setup.bash
export ROS_IP="130.209.252.206"
export ROS_MASTER_URI="http://cornwall:11311"
rosrun baxter_tools enable_robot.py -e

source opt/ros/galactic/setup.bash
source ros2_bridge_ws/install/setup.bash
ros2 run ros1_bridge dynamic_bridge

TERMINAL 2
ros2 run baxter_joint_controller controller

TERMINAL 3
ros2 launch baxter_moveit_ros2 move_group.launch.py

TERMINAL 4
ros2 launch baxter_moveit_ros2 demo.launch.py
