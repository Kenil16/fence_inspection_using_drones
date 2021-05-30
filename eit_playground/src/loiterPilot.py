#!/usr/bin/env python

import rospy
from math import pi, radians, degrees, tan, sin, cos
import mavros as mav
import mavros.utils
import mavros.setpoint as mavSP

import mavros_msgs.msg
import mavros_msgs.srv

from std_msgs.msg import (String, Int8, Float64)
from tf.transformations import (euler_from_quaternion, quaternion_from_euler)
from geometry_msgs.msg import Quaternion

mavros.set_namespace('mavros')

onB_StateSub = '/onboard/state'
targetWP     = '/onboard/setpoint/loiter'
commandSub   = '/gcs/command'

debug = True
class loiterPilot(): 
    def __init__(self):
        ''' ROS node init '''
        rospy.init_node('loiterPilot')
        self.rate = rospy.Rate(20)

        ''' General '''
        self.enable = False
        self.uavHead = 0.0
        self.headingMode = False        
        self.loiterPos = mavSP.PoseStamped()
        self.curPos = mavSP.PoseStamped()

        ''' Subsribers '''
        # Onboard state
        rospy.Subscriber(onB_StateSub, String, self._cb_onStateChange)

        # PX4 local position
        rospy.Subscriber(
            mavros.get_topic('local_position', 'pose'),
            mavSP.PoseStamped, 
            self._cb_onPositionChange)

        # PX4 global compass heading (GPS)
        rospy.Subscriber(
            mavros.get_topic('global_position','compass_hdg'), 
            Float64, 
            self._cb_onHeadingUpdate)

        # Ground control commands
        rospy.Subscriber(commandSub, Int8, self._cb_onCommand)

        ''' Publishers '''
        # Target waypoint for loiter pilot    
        self.loiterPub = rospy.Publisher(
            targetWP, 
            mavSP.PoseStamped, 
            queue_size=10)
      
        rospy.loginfo('Loiter: LoiterPilot Ready')

    ''' Core Functions '''
    def _pubMsg(self, msg, topic):
        if self.headingMode:
            f_ID="base_link"
        else:
            f_ID="att_pose"
        msg.header = mavros.setpoint.Header(
            frame_id=f_ID,
            stamp=rospy.Time.now())

        topic.publish(msg)
        self.rate.sleep()

    ''' Functions '''
    def adjustYaw(self, angle=5.0): 
        (_, _, yaw) = euler_from_quaternion(
            [self.loiterPos.pose.orientation.x, 
            self.loiterPos.pose.orientation.y, 
            self.loiterPos.pose.orientation.z, 
            self.loiterPos.pose.orientation.w])

        yaw += radians(angle)
        orientAdj = quaternion_from_euler(0, 0, yaw)

        self.loiterPos.pose.orientation = Quaternion(*orientAdj)
        
    def setBearing(self, bearing=0.0):
        orientAdj = quaternion_from_euler(0, 0, radians(bearing))

        desiredBearing = mavSP.PoseStamped()
        desiredBearing.pose.orientation = Quaternion(*orientAdj)
        
        return desiredBearing.pose.orientation

    ''' Subscriber callbacks '''
    def _cb_onCommand(self, msg):
        
        command = str(chr(msg.data))
        command.lower()
        if self.enable:
            if command == 'w':
                self.loiterPos.pose.position.x += 0.5 
            if command == 's':
                self.loiterPos.pose.position.x -= 0.5
            if command == 'a':
                self.loiterPos.pose.position.y += 0.5
            if command == 'd':
                self.loiterPos.pose.position.y -= 0.5    
            if command == 'z':
                self.loiterPos.pose.position.z += 0.5 
            if command == 'x':
                self.loiterPos.pose.position.z -= 0.5
            if command == 'q':
                self.adjustYaw(5.0)
            if command == 'e':
                self.adjustYaw(-5.0)
        else:
            options = "wasdqezx" # options as above
            if command in options: 
                rospy.logwarn('Loiter: Pilot not enabled') 
                
    def _cb_onHeadingUpdate(self,msg):
        self.uavHead = msg.data

    def _cb_onStateChange(self, msg):
        if msg.data == 'loiter':
            rospy.loginfo('Loiter: Enabled')
            self.loiterPos = self.curPos
            # self.loiterPos.pose.orientation = self.setBearing(self.uavHead)
            self.enable = True
        else:
            if self.enable:
                rospy.loginfo('Loiter: Disabled')
            self.enable = False
        
    def _cb_onPositionChange(self,msg):
        self.curPos = msg

    def run(self):

        while not rospy.is_shutdown():
            if self.enable:
                self._pubMsg(self.loiterPos, self.loiterPub)
                self.rate.sleep()

if __name__ == "__main__":
    LP = loiterPilot()
    LP.run()
