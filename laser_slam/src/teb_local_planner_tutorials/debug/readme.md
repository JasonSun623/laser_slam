## debud commands
1>nav pause:
	$rostopic pub -1 /app_pub std_msgs/String -- '"{\"pub_name\":\"nav_goal_pause\"}"'
2>nav resume after paused:
	$rostopic pub -1 /app_pub std_msgs/String -- '"{\"pub_name\":\"nav_goal_resume\"}"'
3>pub backward vel through topic "/android_joystick_cmd_vel":
	$rostopic pub -r 1 /android_joystick_cmd_vel geometry_msgs/Twist -- '[-0.5, 0.0, 0.0]' '[0.0, 0.0, 0.0]'

## simulatind launch
	roslaunch teb_local_planner_tutorials cookyplus_simulate.launch


** ATTENTION **
#setting up simulating env according to the web at first time:
	http://wiki.ros.org/teb_local_planner/Tutorials/Setup%20and%20test%20Optimization
