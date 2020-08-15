#! /usr/bin/env python
import rospy
from std_msgs.msg import *
from geometry_msgs.msg import *
from control_msgs.msg import *
from trajectory_msgs.msg import *
import time

joints_pub = rospy.Publisher('/arm_effort_controller/command', JointTrajectory, queue_size=10)
test_angle = 0

def goal_callback(data):
    print(data)

def arm_state(data):
    global test_angle
    # print (data)
    cmd = JointTrajectory()
    cmd.joint_names = ["soulder_base_joint", "shoulder_forearm_joint", "arm_forearm_joint", "roll_arm_joint", "yaw_roll_joint", "pitch_yaw_joint"]
    cmd.points.append(data.actual)
    cmd.points[0].accelerations = [1]*6
    cmd.points[0].velocities = [1]*6
    cmd.points[0].time_from_start = rospy.Duration(0.01)
    test_angle += 0.1
    cmd.points[0].positions = (test_angle, 0.8555191017805814, 0.27609019070510943, -0.09875712992764463, -0.042817870735770036, 0.08739832076399434)
    joints_pub.publish(cmd)
    print(cmd)

    time.sleep(1)

def butia_arm_inverse_kinematics():    
    rospy.init_node("butia_arm_inverse_kinematics", anonymous=False)

    rospy.Subscriber('/butia_arm_inverse_kinematics/goal', Twist, goal_callback)

    rospy.Subscriber('/arm_effort_controller/state', JointTrajectoryControllerState, arm_state)
    
    rospy.spin()

if __name__ == "__main__": 
    butia_arm_inverse_kinematics()     