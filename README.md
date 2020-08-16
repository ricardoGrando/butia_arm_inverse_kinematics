# butia_arm_inverse_kinematics

### Depends on:

	roslaunch doris_moveit_config gazebo.launch 

### How to run:

	rosrun butia_arm_inverse_kinematics butia_arm_inverse_kinematics.py

### How to test a target pose:

	 rostopic pub /butia_arm_inverse_kinematics/goal geometry_msgs/Twist "linear: x: 0.40 y: -0.0 z: 0.1 angular: x: -1.5 y: -0.7 z: 1.65"

 
