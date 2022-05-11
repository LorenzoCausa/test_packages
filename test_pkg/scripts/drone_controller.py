#!/usr/bin/env python3

# IMPORTS
import rospy
import roslib

from geometry_msgs.msg import Pose 
from my_custom_interfaces.msg import Drone_cmd
from std_msgs.msg import Float32

# GLOBAL VARIABLES
x=float(0)
y=float(0)
angle=float(0)
ground_distance=float(3)


# SUBSCRIBERs CALLBACK
def callback_loc(pose):
    global x,y,angle
    x=pose.position.x
    y=pose.position.y
    angle=pose.orientation.z-90

def callback_ground(distance):
    global ground_distance
    ground_distance=distance.data

def main():
    rospy.init_node('drone_controller', anonymous=False)
    rospy.Subscriber("localization", Pose, callback_loc,queue_size=1)
    rospy.Subscriber("ground_distance", Float32, callback_ground,queue_size=1)
    command_pub=rospy.Publisher("command", Drone_cmd, queue_size=1)

    cmd=Drone_cmd()
    rate = rospy.Rate(10) # 10hz 

    while not rospy.is_shutdown():
        cmd.yaw=-angle/10 # - may be due to the inverted image of the simulation
        cmd.throttle=3-ground_distance
        cmd.pitch=-x/200
        command_pub.publish(cmd)
        print("x:",x,", y:",y,", angle:",angle,", ground distance:",ground_distance)
        rate.sleep()

if __name__ == "__main__":
    main()
