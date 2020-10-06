#!/usr/bin/env python
 
import rospy
import keyboard
from geometry_msgs.msg import Twist

def send_cmd_vel():
    pub = rospy.Publisher('rexrov/cmd_vel', Twist, queue_size=10)
    rospy.init_node('cmd_vel_pub')
    rate = rospy.Rate(10)
    # v.linear.x = 0
    # v.linear.y = 0
    # v.linear.z = 0
    # v.angular.x = 1
    # v.angular.y = 0
    # v.angular.z = 0
    while not rospy.is_shutdown():
        v = Twist()
        v.angular.x = 1
        if keyboard.is_pressed('w'):
            v.linear.x = 1
        elif keyboard.is_pressed('s'):
            v.linear.x = -1
        if keyboard.is_pressed('a'):
            v.angular.z = 1
        elif keyboard.is_pressed('d'):
            v.angular.z = -1
        if keyboard.is_pressed('up'):
            v.linear.z = 1
        elif keyboard.is_pressed('down'):
            v.linear.z = -1
        pub.publish(v)
        rate.sleep()

if __name__ == '__main__':
    try:
        send_cmd_vel()
    except rospy.ROSInterruptException:
        pass
