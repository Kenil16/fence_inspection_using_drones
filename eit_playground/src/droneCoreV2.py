#!/usr/bin/env python

import numpy as np
import os

import rospy

import mavros as mav
import mavros.utils
import mavros.command as mavCMD
import mavros.setpoint as mavSP

import mavros_msgs.msg
import mavros_msgs.srv

from sensor_msgs.msg import NavSatFix
from std_msgs.msg import (String, Int8, Float64, Bool)
from geometry_msgs.msg import PoseStamped, Quaternion
from tf.transformations import quaternion_from_euler, euler_from_quaternion

mavros.set_namespace('mavros')

onB_StateSub     = '/onboard/state'
mh_enableSub     = '/onboard/enableMH'
wpCompleteTopic  = '/onboard/check/WPSuccess'

commandSub       = '/gcs/command'

class droneCore():
    def __init__(self):
        ''' ROS node init '''
        rospy.init_node('DroneCoreNode')
        self.rate = rospy.Rate(20)
        
        ''' General variables ''' 
        self.MH_enabled = False
        self.sysState = 'idle' 
        self.MAVROS_State = mavros_msgs.msg.State()
        self.MAVROS_PrevState = mavros_msgs.msg.State()
        self.isAirbourne = False
        
        self.uavGPSPos = None
        self.uavLocalPos = mavSP.PoseStamped()
        self.uavLocalSetpoint = mavSP.PoseStamped()
        self.uavHdg = None  

        ''' Subscribers '''
        # KillSwitch
        #rospy.Subscriber(isolatedSub, Bool, self._cb_onKillSwitch)

        # PX4 state 
        rospy.Subscriber(
            mavros.get_topic('state'),
            mavros_msgs.msg.State,
            self._cb_uavState) # 
        
        # PX4 global position (GPS)
        rospy.Subscriber(
            mavros.get_topic('global_position','global'),
            NavSatFix, 
            self._cb_SatFix)

        # PX4 setpoint local 
        rospy.Subscriber(
            mavros.get_topic('setpoint_position','local'),
            mavSP.PoseStamped, 
            self._cb_onSetpointChange)

        # PX4 local position
        rospy.Subscriber(
            mavros.get_topic('local_position', 'pose'),
            mavSP.PoseStamped, 
            self._cb_onPositionChange)

        # PX4 global compass heading (GPS)
        rospy.Subscriber(
            mavros.get_topic('global_position', 'compass_hdg'),
            Float64, 
            self._cb_headingUpdate)

        # Ground control commands
        rospy.Subscriber(
            commandSub,
            Int8,
            self._cb_onCommand)

        ''' Publishers '''
        # Onboard state
        self.statePub = rospy.Publisher(
            onB_StateSub, 
            String,
            queue_size=1)

        # Enable message handler
        self.enableMHPub = rospy.Publisher(
            mh_enableSub,
            Bool,
            queue_size=1)

        # Waypoint complete
        self.wpCompletePub = rospy.Publisher(
            wpCompleteTopic, 
            Bool, 
            queue_size=1)
        
        #self.llpPub = rospy.Publisher(isolatedSub, Bool, queue_size=1)
        self.spLocalPub = mavSP.get_pub_position_local(queue_size=5)

        ''' Services '''
        self.setMode = rospy.ServiceProxy(
            '/mavros/set_mode', 
            mavros_msgs.srv.SetMode)
        #self.enableTakeoff = rospy.ServiceProxy(
            #'/mavros/cmd/takeoff', 
            #mavros_msgs.srv.CommandTOL)

        # Perform MAVROS handshake   
        self._mavrosHandshake()

        # Param update
        #os.system('rosrun mavros mavparam set MIS_TAKEOFF_ALT 1.5')

    ''' Core Functions '''
    def _mavrosHandshake(self): 
        rospy.loginfo('DroneCore: Waiting for MAVROS Connection.')
        i=0
        time = rospy.Time.now()
        for i in range(0,3):
            print'.',
            if self.MAVROS_State.connected:
                rospy.loginfo("DroneCore: MAVROS Connected!")
                break
            rospy.sleep(1)
        if not self.MAVROS_State.connected:
            errorMsg = "DroneCore: MAVROS not connected!"
            rospy.logfatal(errorMsg)
            rospy.signal_shutdown(errorMsg)
    
    # Generic function to publish a message
    def _pubMsg(self, msg, topic):
        msg.header = mavSP.Header(
            frame_id="att_pose",
            stamp=rospy.Time.now())

        topic.publish(msg)
        self.rate.sleep()

    # Fuction for updating onboard state
    def _setState(self, state):
        self.sysState = state
        if state == 'idle' or state == 'takeoff' or state == 'mission':
            self.enableMHPub.publish(False)
        elif not self.MH_enabled:
            self.enableMHPub.publish(True)
            self.MH_enabled = True
        self.statePub.publish(state)
        rospy.loginfo('DroneCore: state = {}'.format(state))

    # Function that waits for PX4 mode change
    def _waitForPX4Mode(self, mode, time):
        for i in range(0,time):
            self.rate.sleep()
            if self.MAVROS_State.mode == mode:
                return True

        rospy.logwarn('DroneCore: PX4 mode change timeout')
        return False

    ''' Subscriber callbacks '''
    #def _cb_onKillSwitch(self,msg):
        #if msg.data==True:
            #self.enableMHPub.publish(False)
            #self.MH_enabled = False
            #self.statePub.publish('isolate')
    def _cb_onPositionChange(self, msg):
        self.uavLocalPos = msg
        pass

    def _cb_onSetpointChange(self, msg):
        self.uavLocalSetpoint = msg
        pass

    def _cb_onCommand(self, msg):
        command = str(chr(msg.data))
        command.lower()
    
        if command == 't': # Takeoff
            self.droneTakeoff()
        if command == 'o': # Offboard control
            if self.sysState == 'idle':
                self.enableMHPub.publish(True)

                self._setState('loiter')
                for i in range(0,3):
                    self.setMode(0,'OFFBOARD')
                    if self.MAVROS_State.mode == 'OFFBOARD':
                        break
                #rospy.loginfo('DroneCore: PX4 mode = OFFBOARD')
            else:
                self._setState('idle')
                for i in range(0,3):
                    self.setMode(0,'AUTO.LOITER')
                    if self.MAVROS_State.mode == 'AUTO.LOITER':
                        break
                #rospy.loginfo('DroneCore: PX4 mode = AUTO.LOITER')
        if command == 'h': # Returns the drone to home
            self._setState('idle')
            self.setMode(0,'AUTO.RTL')
        if command == 'v': # Perform vision guided landing
            self._setState('vision_land')
        if command == 'm': # Execute mission
            #if self.MAVROS_State.mode != 'OFFBOARD':
                #rospy.logwarn('DroneCore: OFFBOARD not enabled')
            #else:
            self._setState('mission')
            #self.droneFenceDetectionMission()
        if command == 'r': # Reset ROS framework
            self._setState('idle')
            self.isAirbourne = False
            rospy.loginfo('DroneCore: Resetting ROS framwork')
        if command == 'k': # Kill drone
            self._setState('idle')  
            self.isAirbourne = False          
            #TODO: Implement PX4 kill switch
        if command == 'l': # Land at current location
            self._setState('land')
            self.setMode(0,'AUTO.LAND')
            self.enableMHPub.publish(False)
            while self.MAVROS_State.armed:
                self.rate.sleep()
            self.isAirbourne = False
            self._setState('idle')

    def _cb_SatFix(self, msg):
        self.uavGPSPos = msg
    
    def _cb_headingUpdate(self,msg):
        self.uavHdg = msg
    
    def _cb_uavState(self, msg):
        self.MAVROS_PrevState = self.MAVROS_State
        self.MAVROS_State = msg
        if self.MAVROS_PrevState.mode != self.MAVROS_State.mode:
            rospy.loginfo('DroneCore: PX4 mode = {}'.format(self.MAVROS_State.mode))
    
        #if self.sysState != 'idle' and self.sysState != 'takeoff' and self.MAVROS_State.mode != 'OFFBOARD':
            #rospy.logwarn("DroneCore: System enabled, but drone is in manual control. Disabling Message Handler")
            #self._setState('idle')
        pass

    ''' Functions '''
    def waypointCheck(self, threshold=0.15):
        pos = np.array((self.uavLocalPos.pose.position.x,
            self.uavLocalPos.pose.position.y,
            self.uavLocalPos.pose.position.z))

        setpoint = np.array((self.uavLocalSetpoint.pose.position.x,
            self.uavLocalSetpoint.pose.position.y,
            self.uavLocalSetpoint.pose.position.z))        

        return np.linalg.norm(setpoint - pos) < threshold

    def droneTakeoff(self):
        if self.isAirbourne == False or self.sysState == 'idle':
            if not self.MAVROS_State.armed:
                mavCMD.arming(True)
                rospy.loginfo('DroneCore: Arming')

            self._setState('takeoff')
            self.setMode(0, 'AUTO.TAKEOFF')
            self._waitForPX4Mode('AUTO.TAKEOFF',30)

            self.isAirbourne = True
            while (self.MAVROS_State.mode != 'AUTO.LOITER'):
                self.rate.sleep()

            self.enableMHPub.publish(True)
            self._setState('loiter')

            for i in range(0,3):
                self.setMode(0,'OFFBOARD')
                self.rate.sleep()
                if self.MAVROS_State.mode == 'OFFBOARD':
                    break

    def run(self):
        while not rospy.is_shutdown():
            self.wpCompletePub.publish(self.waypointCheck())
            self.rate.sleep()
        pass

if __name__ == "__main__":
    dc = droneCore()
    dc.run()
