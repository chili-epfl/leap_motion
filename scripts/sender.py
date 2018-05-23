#!/usr/bin/env python
__author__ = 'flier'

import argparse

import rospy
import leap_interface
from leap_motion.msg import leap
from leap_motion.msg import leapros
from visualization_msgs.msg import MarkerArray, Marker

FREQUENCY_ROSTOPIC_DEFAULT = 0.01
NODENAME = 'leap_pub'
PARAMNAME_FREQ = 'freq'
PARAMNAME_FREQ_ENTIRE = '/' + NODENAME + '/' + PARAMNAME_FREQ

def sender():
    '''
    This method publishes the data defined in leapros.msg to /leapmotion/data
    '''
    rospy.loginfo("Parameter set on server: PARAMNAME_FREQ={}".format(rospy.get_param(PARAMNAME_FREQ_ENTIRE, FREQUENCY_ROSTOPIC_DEFAULT)))

    li = leap_interface.Runner()
    li.setDaemon(True)
    li.start()
    # pub     = rospy.Publisher('leapmotion/raw',leap)
    pub_ros   = rospy.Publisher('leapmotion/data',leapros, queue_size=2)
    pub_ros_viz   = rospy.Publisher('leapmotion/data_viz',MarkerArray, queue_size=2)
    rospy.init_node(NODENAME)


    while not rospy.is_shutdown():

        #rospy.loginfo( "hello world %s"%rospy.get_time())
        hand_direction_   = li.get_hand_direction()
        hand_normal_      = li.get_hand_normal()
        hand_palm_pos_    = li.get_hand_palmpos()
        hand_pitch_       = li.get_hand_pitch()
        hand_roll_        = li.get_hand_roll()
        hand_yaw_         = li.get_hand_yaw()

        msg = leapros()
        msg.direction.x = hand_direction_[0]
        msg.direction.y = hand_direction_[1]
        msg.direction.z = hand_direction_[2]
        msg.normal.x = hand_normal_[0]
        msg.normal.y = hand_normal_[1]
        msg.normal.z = hand_normal_[2]
        msg.palmpos.x = hand_palm_pos_[0]
        msg.palmpos.y = hand_palm_pos_[1]
        msg.palmpos.z = hand_palm_pos_[2]
        msg.ypr.x = hand_yaw_
        msg.ypr.y = hand_pitch_
        msg.ypr.z = hand_roll_

        fingerNames = ['thumb', 'index', 'middle', 'ring', 'pinky']
        fingerPointNames = ['metacarpal', 'proximal',
                            'intermediate', 'distal', 'tip']

        markerArray = MarkerArray()
        counter = 0

        for fingerName in fingerNames:
            for fingerPointName in fingerPointNames:
                pos = li.get_finger_point(fingerName, fingerPointName)
                #rospy.loginfo(pos)


                # todo push back the point
                for iDim, dimName in enumerate(['x', 'y', 'z']):
                    setattr(getattr(msg, '%s_%s' % (fingerName, fingerPointName)),
                            dimName, pos[iDim])
                    marker = Marker()


                    marker.header.frame_id = "/leap_optical_frame";
                    marker.header.stamp = rospy.Time.now()
                    marker.id = counter
                    counter+=1
                    marker.type = Marker.SPHERE
                    marker.scale.x = marker.scale.y = marker.scale.z = 0.2
                    marker.color.r = .0
                    marker.color.g = 1.0
                    marker.color.b = 1.0
                    marker.color.a = 0.7
                    marker.action = Marker.ADD
                    marker.lifetime  = rospy.rostime.Duration(0.1)
                    marker.pose.orientation.w = 1.0
                    marker.pose.position.x = pos[0] /100
                    marker.pose.position.y = pos[1] /100
                    marker.pose.position.z = pos[2] /100
                    markerArray.markers.append(marker)

        pub_ros_viz.publish(markerArray)
        # We don't publish native data types, see ROS best practices
        # pub.publish(hand_direction=hand_direction_,hand_normal = hand_normal_, hand_palm_pos = hand_palm_pos_, hand_pitch = hand_pitch_, hand_roll = hand_roll_, hand_yaw = hand_yaw_)
        pub_ros.publish(msg)
        rospy.sleep(rospy.get_param(PARAMNAME_FREQ_ENTIRE, FREQUENCY_ROSTOPIC_DEFAULT))


if __name__ == '__main__':
    try:
        sender()
    except rospy.ROSInterruptException:
        pass
