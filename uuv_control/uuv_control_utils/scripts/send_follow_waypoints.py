#!/usr/bin/env python

import roslib
import rospy

from math import *

from uuv_control_msgs.msg import WaypointSet, Waypoint
from uuv_control_msgs.srv import AddWaypoint, InitWaypointSet, GoTo
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Time

def follow_pose_callback(data):
    global wp_list
    if (len(wp_list) == 0 or (sqrt(pow(data.pose.pose.position.x - wp_list[-1].point.x, 2) +
        pow(data.pose.pose.position.y - wp_list[-1].point.y, 2) +
        pow(data.pose.pose.position.z - wp_list[-1].point.z, 2)) >= 1)):

        wp = Waypoint()
        wp.max_forward_speed = max_forward_speed
        wp.heading_offset = heading_offset
        wp.use_fixed_heading = use_fixed_heading
        wp.radius_of_acceptance = radius_of_acceptance
        wp.header.frame_id = 'world'
        wp.point = data.pose.pose.position

        wp_list.append(wp)
        rospy.loginfo('# waypoints = %d' % len(wp_list))

        # success = init_wp(Time(rospy.Time.from_sec(rospy.Time.now().to_sec())),
        #               True,
        #               wp_list,
        #               max_forward_speed,
        #               heading_offset,
        #               String(interpolator))

        #response = add_wp(wp)
        # response = go_to(wp, max_forward_speed, interpolator)
        # if not response.success:
        #     rospy.loginfo('Failed to add a new waypoint')
        # else:
        #     wp_list = response.waypoints
        #     rospy.loginfo('Added a waypoint')
        #     rospy.loginfo('# waypoints = %d' % len(wp_list))
        #     rospy.loginfo(str(wp_list))

def self_pose_callback(data):
    global wp_list
    if (len(wp_list) > 1):
        # reached the first waypoint in the waypoint list
        # going to the next waypoint
        if (sqrt(pow(data.pose.pose.position.x - wp_list[0].point.x, 2) +
            pow(data.pose.pose.position.y - wp_list[0].point.y, 2) +
            pow(data.pose.pose.position.z - wp_list[0].point.z, 2)) <= 0.1):
            wp_list.pop(0)

            try:
                go_to = rospy.ServiceProxy(
                    '/%s/go_to' % uuv_name,
                    GoTo)
            except rospy.ServiceException as e:
                raise rospy.ROSException('Service call for go to failed, error=%s', str(e))
            
            wp = wp_list[0]

            response = go_to(wp, max_forward_speed, interpolator)
            if not response:
                rospy.loginfo('Failed to start a new trajectory')
            else:
                rospy.loginfo('Started a new trajectory')
    
            # try:
            #     init_wp = rospy.ServiceProxy(
            #         '/%s/start_waypoint_list' % uuv_name,
            #         InitWaypointSet)
            # except rospy.ServiceException as e:
            #     raise rospy.ROSException('Service call for initialize waypoint set failed, error=%s', str(e))
            
            # success = init_wp(Time(rospy.Time.from_sec(rospy.Time.now().to_sec())),
            #                 True,
            #                 wp_list,
            #                 max_forward_speed,
            #                 heading_offset,
            #                 String(interpolator))

            # if success:
            #     rospy.loginfo(len(wp_list))
            # else:
            #     rospy.loginfo('Failed to remove an old waypoint')

if __name__ == '__main__':
    rospy.init_node('follow_waypoint_publisher')
    #listener = tf.TransformListener()
    rate = rospy.Rate(0.5)

    if rospy.is_shutdown():
        rospy.logerr('ROS master not running!')
        sys.exit(-1)

    uuv_name = rospy.get_param('~uuv_name')
    follow_name = rospy.get_param('~follow_name')
    max_forward_speed = rospy.get_param('~max_forward_speed')
    heading_offset = rospy.get_param('~heading_offset')
    use_fixed_heading = rospy.get_param('~use_fixed_heading')
    radius_of_acceptance = rospy.get_param('~radius_of_acceptance')

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

    # try:
    #     rospy.wait_for_service('/%s/start_waypoint_list' % uuv_name, timeout=30)
    # except rospy.ROSException:
    #     raise rospy.ROSException('Service start_waypoint_list not available! Closing node...')

    # try:
    #     rospy.wait_for_service('/%s/add_waypoint' % uuv_name, timeout=30)
    # except rospy.ROSException:
    #     raise rospy.ROSException('Service add_waypoint_not available! Closing node...')

    try:
        rospy.wait_for_service('/%s/go_to' % uuv_name, timeout=30)
    except rospy.ROSException:
        raise rospy.ROSException('Service go_to not available! Closing node...')

    
    # try:
    #     init_wp = rospy.ServiceProxy(
    #         '/%s/start_waypoint_list' % uuv_name,
    #         InitWaypointSet)
    # except rospy.ServiceException as e:
    #     raise rospy.ROSException('Service call for initialize waypoint set failed, error=%s', str(e))

    wp = Waypoint()
    wp.max_forward_speed = max_forward_speed
    wp.heading_offset = heading_offset
    wp.use_fixed_heading = use_fixed_heading
    wp.radius_of_acceptance = radius_of_acceptance
    wp.header.frame_id = 'world'
    wp.point.x = 30
    wp.point.y = 0
    wp.point.z = -20

    wp_list = [wp]
    success = True

    # success = init_wp(Time(rospy.Time.from_sec(start_time)),
    #                   start_now,
    #                   wp_list,
    #                   max_forward_speed,
    #                   heading_offset,
    #                   String(interpolator))
    if success:
        # try:
        #     rospy.wait_for_service('/%s/add_waypoint' % uuv_name, timeout=30)
        # except rospy.ROSException:
        #     raise rospy.ROSException('Service not available! Closing node...')

        rospy.Subscriber('/%s/pose_gt' % follow_name,
            Odometry,
            follow_pose_callback)
        
        rospy.Subscriber('/%s/pose_gt' % uuv_name,
            Odometry,
            self_pose_callback)
        
        rospy.spin()

        # while not rospy.is_shutdown():
        #     if (len(wp_list) > 100):
        #         wp_list.pop(0)
        #         success = init_wp(Time(rospy.Time.from_sec(rospy.Time.now().to_sec())),
        #                         True,
        #                         wp_list,
        #                         max_forward_speed,
        #                         heading_offset,
        #                         String(interpolator))

        #         if success:
        #             rospy.loginfo(len(wp_list))
        #         else:
        #             rospy.loginfo('Failed to remove an old waypoint')
        #         rate.sleep()      
    # else:
    #     rospy.loginfo('Failed to send waypoints')