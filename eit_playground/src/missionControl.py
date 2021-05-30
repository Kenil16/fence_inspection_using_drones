#!/usr/bin/env python

import rospy
from math import pi, radians, degrees, tan, sin, cos
import mavros as mav
import mavros.utils
import mavros.setpoint as mavSP

import mavros_msgs.msg
import mavros_msgs.srv

from std_msgs.msg import (String, Int8, Float64, Bool)
from tf.transformations import (euler_from_quaternion, quaternion_from_euler)
from geometry_msgs.msg import Quaternion

import numpy as np

import os

from missionReader import missionReader

mavros.set_namespace('mavros')

onB_StateSub    = '/onboard/state'
onB_SubStateSub = '/onboard/substate'
targetWP        = '/onboard/setpoint/mission'
wpCompleteTopic = '/onboard/check/WPSuccess'
commandSub      = '/gcs/command'

class missionControl(): 
    def __init__(self):
        ''' ROS node init '''
        rospy.init_node('missionControl')
        self.rate = rospy.Rate(20)

        ''' General '''
        self.enable = False
        self.onState = 'idle'
        self.waypoint = mavSP.PoseStamped()
        self.forward = True        
        self.wpComplete = False

        self.mission = missionReader('mission_00.txt')   

        ''' Subsribers '''
        # Onboard state
        rospy.Subscriber(onB_StateSub, String, self._cb_onStateChange)

        rospy.Subscriber(wpCompleteTopic, Bool, self._cb_onWPComplete)

        ''' Publishers '''
        # Target waypoint for loiter pilot    
        self.waypointPub = rospy.Publisher(
            targetWP, 
            mavSP.PoseStamped, 
            queue_size=5)
      
        # Onboard State
        self.statePub = rospy.Publisher(
            onB_StateSub,
            String,
            queue_size=1)        

        # Onboard Substate
        self.substatePub = rospy.Publisher(
            onB_SubStateSub,
            String,
            queue_size=1)

        # Control commands
        self.commandPub = rospy.Publisher(
            commandSub,
            Int8,
            queue_size=1)

        rospy.loginfo('Mission: MissionControl Ready')

    ''' Core Functions '''
    def _pubMsg(self, msg, topic):
        f_ID="att_pose"
        msg.header = mavros.setpoint.Header(
            frame_id=f_ID,
            stamp=rospy.Time.now())

        topic.publish(msg)
        self.rate.sleep()

    ''' Functions '''
    def executeMission(self):
        if self.mission.missionState == 'idle':
            self.mission.missionState = 'getNextCommand'

        elif self.mission.missionState == 'getNextCommand':
            self.mission.getNext(self.forward)
            self.forward = True

        elif self.mission.missionState == 'sendCommand':
            self.commandPub.publish(ord(self.mission.nextCommand))
            self.mission.missionState = 'waitForState'

        elif self.mission.missionState == 'waitForState':
            if self.onState == self.mission.nextState:
                self.mission.missionState = 'getNextCommand'

        elif self.mission.missionState == 'setState':
            self.statePub.publish(self.mission.nextState)
            self.mission.missionState = 'getNextCommand'

        elif self.mission.missionState == 'setSubState':
            self.substatePub.publish(self.mission.nextSubState)
            self.mission.missionState = 'getNextCommand'

        elif self.mission.missionState == 'updateParam':
            tmp = 'rosrun mavros mavparam set ' + self.mission.nextParam
            tmp = tmp + ' {:f}'.format(float(self.mission.nextParamVal))
            os.system(tmp)
            self.mission.missionState = 'getNextCommand'

        elif self.mission.missionState == 'setWaypoint':
            self.waypoint = self.mission.nextWaypoint

            self.mission.missionState = 'waitForWP'

            if self.onState != 'mission':
                self.forward = False
                self.mission.nextState = 'mission'
                self.mission.missionState = 'setState'
            self.rate.sleep()

        elif self.mission.missionState == 'waitForWP':
            if self.wpComplete == True:
                self.mission.missionState = 'getNextCommand'

        elif self.mission.missionState == 'complete':
            rospy.loginfo('Mission: Mission complete')
            self.enable = False
            self.mission.missionState = 'idle'

        
        if self.onState != 'idle':
            self._pubMsg(self.waypoint, self.waypointPub)


    ''' Subscriber callbacks '''
    def _cb_onStateChange(self, msg):
        self.onState = msg.data
        if msg.data == 'mission':
            rospy.loginfo('Mission: Enabled')
            self.enable = True
        elif msg.data == 'idle':
            if self.enable:
                rospy.loginfo('Mission: Disabled')
            self.enable = False

    def _cb_onWPComplete(self,msg):
        self.wpComplete = msg.data
        pass

    def run(self):

        while not rospy.is_shutdown():
            if self.enable:
                self.executeMission()
                self.rate.sleep()

if __name__ == "__main__":
    mc = missionControl()
    mc.run()
