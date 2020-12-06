#!/usr/bin/env python

# The following robot will go straight to the leading robot

import roslib
import rospy
import random
import functools

from math import *

from uuv_control_msgs.msg import Waypoint
from uuv_control_msgs.srv import GoTo
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Time

def pose_delay_callback(data, event):
    wp = Waypoint()
    wp.max_forward_speed = max_forward_speed
    wp.heading_offset = heading_offset
    wp.use_fixed_heading = use_fixed_heading
    wp.radius_of_acceptance = radius_of_acceptance
    wp.header.frame_id = 'world'
    wp.point = data.pose.pose.position

    try:
        go_to = rospy.ServiceProxy('go_to', GoTo)
    except rospy.ServiceException as e:
        raise rospy.ROSException('Service call for go to failed, error=%s', str(e))

    response = go_to(wp, max_forward_speed, interpolator)
    if not response:
        rospy.loginfo('Failed to start a new trajectory')
    else:
        rospy.loginfo('Started a new trajectory')

def follow_pose_callback(data):
    # use the last_position to control the trajectory update frequency
    # only accepts new postion when it is at least 
    # 10 meters from the last received position
    # of course this can be replaced by a timer
    if last_pos == None or sqrt(pow(data.pose.pose.position.x - last_pos.pose.pose.position.x) +
            pow(data.pose.pose.position.y - last_pos.pose.pose.position.y) +
            pow(data.pose.pose.position.z - last_pos.pose.pose.position.z)) > 10:

        if random.randrange(0, 100) > success_rate:
            return  # message fails to pass through

        # add a delay for the following position message
        rospy.Timer(rospy.Duration(10.0),
                    functools.partial(pose_delay_callback, data),
                    oneshot = True)

if __name__ == '__main__':
    rospy.init_node('simple_follow_waypoint_publisher')

    rate = rospy.Rate(0.5)

    if rospy.is_shutdown():
        rospy.logerr('ROS master not running!')
        sys.exit(-1)

    max_forward_speed = rospy.get_param('~max_forward_speed')
    heading_offset = rospy.get_param('~heading_offset')
    use_fixed_heading = rospy.get_param('~use_fixed_heading')
    radius_of_acceptance = rospy.get_param('~radius_of_acceptance')
    success_rate = rospy.get_param('~success_rate')

    # If no start time is provided: start *now*.
    start_time = rospy.Time.now().to_sec()
    start_now = True
    if rospy.has_param('~start_time'):
        start_time = rospy.get_param('~start_time')
        if start_time < 0.0:
            rospy.logerr('Negative start time, setting it to 0.0')
            start_time = 0.0
            start_now = True
        else:
            start_now = False
    else:
        start_now = True

    rospy.loginfo('Start time=%.2f s' % start_time)

    interpolator = rospy.get_param('~interpolator', 'lipb')

    try:
        rospy.wait_for_service('go_to', timeout=30)
    except rospy.ROSException:
        raise rospy.ROSException('Service go_to not available! Closing node...')

    last_pos = None

    rospy.Subscriber('follow/pose_gt',
            Odometry,
            follow_pose_callback)
        
    rospy.spin()