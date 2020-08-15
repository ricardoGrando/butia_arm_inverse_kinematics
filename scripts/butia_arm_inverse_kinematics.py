#! /usr/bin/env python
import rospy
from std_msgs.msg import *
from geometry_msgs.msg import *
from control_msgs.msg import *
from trajectory_msgs.msg import *
import time
from numpy import *
import math
from kinematics import *

joints_pub = rospy.Publisher('/arm_effort_controller/command', JointTrajectory, queue_size=10)
angle_offset = array( [0.0, -0.5, -2.4, 0.0, 0.1, 1.6] )
# starting_position = (0.0, math.pi/3, 3*math.pi/4, 0.0, 0.0, 0.0)
target_position = np.zeros(6)
current_angles = np.zeros(6)
current_angles[0] = 0.1
current_angles[1] = 0.1

# [ 0.39301719  0.0560871   0.15478628 -1.49747691 -0.71345515  1.64174941]

step = 0.0001

def goal_callback(data):
    global target_position
    global step 
    global current_angles
    target_position[0] = data.linear.x
    target_position[1] = data.linear.y
    target_position[2] = data.linear.z
    target_position[3] = data.angular.x
    target_position[4] = data.angular.y
    target_position[5] = data.angular.z

    atual_position = forwardKinematics(current_angles)

    print(atual_position)

    angles = current_angles.copy()
    
    while(max(abs(target_position - atual_position)) > 0.01):        

        atual_position = forwardKinematics(angles)
        distance = target_position - atual_position
        print("##########################################")
        print(atual_position)

        J = calc_jacobian(angles)
        J_inv = np.linalg.inv(J)

        delta_end_effector = ((distance)*step)/np.max(max(abs(distance)))

        delta_angles = J_inv.dot(delta_end_effector)

        angles += delta_angles

        print(angles)

        cmd = JointTrajectory()
        cmd.joint_names = ["soulder_base_joint", "shoulder_forearm_joint", "arm_forearm_joint", "roll_arm_joint", "yaw_roll_joint", "pitch_yaw_joint"]
        cmd.points.append(JointTrajectoryPoint())  
        cmd.points[0].accelerations = [1]*6
        cmd.points[0].velocities = [1]*6
        cmd.points[0].time_from_start = rospy.Duration(0.01) 

        cmd.points[0].positions = tuple(angles) + angle_offset

        joints_pub.publish(cmd)

        # time.sleep(0.1)

def arm_state(data):
    global current_angles
    
    current_angles = np.asarray(data.actual.positions - angle_offset)

    # print(current_angles)
   

def butia_arm_inverse_kinematics():    
    rospy.init_node("butia_arm_inverse_kinematics", anonymous=False)

    rospy.Subscriber('/butia_arm_inverse_kinematics/goal', Twist, goal_callback)

    rospy.Subscriber('/arm_effort_controller/state', JointTrajectoryControllerState, arm_state)
    
    rospy.spin()

if __name__ == "__main__": 
    butia_arm_inverse_kinematics() 


