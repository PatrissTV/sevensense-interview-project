
# Build tools
build-all:
	colcon build --symlink-install

build-controller:
	colcon build --packages-select alphasense

# Running the project
spawn:
	ros2 launch irobot_create_gazebo_bringup create3_gazebo.launch.py namespace:=alpha1 x:=0.0 y:=0.0 yaw:=0.0

spawn-second:
	@echo "Make sure you have spawned the first robot before!"
	ros2 launch irobot_create_gazebo_bringup create3_spawn.launch.py namespace:=alpha2 x:=0.0 y:=1.0 yaw:=0.0

launch-alpha1:
	ros2 launch alphasense alphasense.launch.py namespace:=alpha1

launch-alpha2:
	ros2 launch alphasense alphasense.launch.py namespace:=alpha2

# Tools for testing actions
undock:
	ros2 action send_goal /undock irobot_create_msgs/action/Undock "{}"

dock:
	ros2 action send_goal /dock irobot_create_msgs/action/Dock "{}"

navigate-dock:
	ros2 action send_goal /navigate_to_position irobot_create_msgs/action/NavigateToPosition "{achieve_goal_heading: true,goal_pose:{pose:{position:{x: -1.0, y: 0.0, z: 0.0}, orientation:{x: 0.0, y: 0.0, z: 0.0}}}}"
