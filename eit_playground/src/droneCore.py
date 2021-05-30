#!/usr/bin/env python

import numpy as np

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
fence_detection_pub = '/onboard/setpoint/fence_detection'

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
        
        self.fence_detection_pub = rospy.Publisher(fence_detection_pub, PoseStamped, queue_size=1)

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
        self.pub_local_pose = rospy.Publisher('/mavros/setpoint_position/local',PoseStamped,queue_size=10)

        ''' Services '''
        self.setMode = rospy.ServiceProxy(
            '/mavros/set_mode', 
            mavros_msgs.srv.SetMode)

        self.enableTakeoff = rospy.ServiceProxy(
            '/mavros/cmd/takeoff', 
            mavros_msgs.srv.CommandTOL)

        #self.set_home_pos = rospy.ServiceProxy('/mavros/cmd/set_home', mavros_msgs.srv.CommandHome)

        # Perform MAVROS handshake   
        self._mavrosHandshake()


        # Update parameters
        #RTL_RETURN_ALT

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
        if state == 'idle':
            self.enableMHPub.publish(False)
        elif not self.MH_enabled:
            self.enableMHPub.publish(True)
            self.MH_enabled = True
        self.statePub.publish(state)
        rospy.loginfo('DroneCore: state = {}'.format(state))

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
            #if self.sysState == 'takeoff':
                #self._setState('loiter')
        if command == 'o': # Offboard control
            if self.sysState == 'idle':
                self.enableMHPub.publish(True)

                self._setState('loiter')
                for i in range(0,3):
                    self.setMode(0,'OFFBOARD')
                    if self.MAVROS_State.mode == 'OFFBOARD':
                        rospy.loginfo('DroneCore: PX4 mode = OFFBOARD')
                        break
            else:
                self._setState('idle')
                for i in range(0,3):
                    self.setMode(0,'AUTO.LOITER')
                    if self.MAVROS_State.mode == 'AUTO.LOITER':
                        rospy.loginfo('DroneCore: PX4 mode = AUTO.LOITER')
                        break
        if command == 'h': # Returns the drone to home
            pass
        if command == 'v': # Perform vision guided landing
            self._setState('vision_land')
        if command == 'm': # Execute mission
            self.droneFenceDetectionMission()
            #self._setState('mission')
        if command == 'r': # Reset ROS framework
            self._setState('idle')
            self.isAirbourne = False
            rospy.loginfo('DroneCore: Resetting ROS framwork')
        if command == 'k': # Kill drone
            self._setState('idle')  
            self.isAirbourne = False          
            #TODO: Implement PX4 kill switch

    def _cb_SatFix(self, msg):
        self.uavGPSPos = msg
    
    def _cb_headingUpdate(self,msg):
        self.uavHdg = msg
    
    def _cb_uavState(self, msg):
        self.MAVROS_State = msg
        if self.sysState != 'idle' and self.sysState != 'takeoff' and self.MAVROS_State.mode != 'OFFBOARD':
             rospy.logwarn("DroneCore: System enabled, but drone is in manual control. Disabling Message Handler")
             self._setState('idle')
             self.setMode(0,'AUTO.RTL')


#AUTO.PRECLAND AUTO.FOLLOW_TARGET AUTO.RTGS AUTO.LAND AUTO.RTL AUTO.MISSION RATTITUDE AUTO.LOITER STABILIZED AUTO.TAKEOFF OFFBOARD POSCTL ALTCTL AUTO.READY ACRO MANUAL

        pass

    ''' Functions '''
    def waypointCheck(self, threshold=0.25):
        pos = np.array((self.uavLocalPos.pose.position.x,
            self.uavLocalPos.pose.position.y,
            self.uavLocalPos.pose.position.z))

        setpoint = np.array((self.uavLocalSetpoint.pose.position.x,
            self.uavLocalSetpoint.pose.position.y,
            self.uavLocalSetpoint.pose.position.z))        

        return np.linalg.norm(setpoint - pos) < threshold

    def droneTakeoff(self, alt=1.5):
        if self.isAirbourne == False or self.sysState == 'idle':
            if not self.MAVROS_State.armed:
                mavCMD.arming(True)
                rospy.loginfo('DroneCore: Arming')

            preArmMsgs = self.uavLocalPos
            preArmMsgs.pose.position.z = alt
            rospy.loginfo('DroneCore: Takeoff altitude = {} m'.format(preArmMsgs.pose.position.z))

            for i in range(0,50):
                self._pubMsg(preArmMsgs, self.spLocalPub)
                #self.rate.sleep()

            self.setMode(0, 'OFFBOARD')
            rospy.loginfo('DroneCore: PX4 mode = OFFBOARD')

            self._setState('takeoff')
            self.isAirbourne = True
            rospy.loginfo('DroneCore: UAV is airbourne')
            #wait until takeoff has occurred
            while(not self.waypointCheck()):
                self._pubMsg(preArmMsgs, self.spLocalPub)

            rospy.loginfo('DroneCore: Takeoff complete')
            self._setState('loiter')
            self.rate.sleep()
            self.enableMHPub.publish(self.isAirbourne)
    
    def droneFenceDetectionMission(self):
        
        #To change velocity of the drone set the MPC_XY_VEL_MAX, MPC_Z_VEL_MAX_DN and MPC_Z_VEL_MAX_UP parameters
        alt_ = 1
        self.droneTakeoff(alt = alt_)
        
        waypoints = [[-5, 0, alt_], [5, 0, alt_], [0, 0, alt_]]
        angle = Quaternion(*quaternion_from_euler(0, 0, np.deg2rad(90)))

        self._setState('fence_breach_detection')
        for waypoint in waypoints:
            
            preArmMsgs = self.uavLocalPos
            preArmMsgs.pose.position.x = waypoint[0]
            preArmMsgs.pose.position.y = waypoint[1]
            preArmMsgs.pose.position.z = waypoint[2]
            preArmMsgs.pose.orientation = angle
            
            #wait until waypoint reached
            while(not self.waypointCheck()):
                self.fence_detection_pub.publish(preArmMsgs)
                self._pubMsg(preArmMsgs, self.spLocalPub)
            
        self._setState('idle')
        rospy.loginfo('DroneCore: Mission fence detection complete')

    def run(self):
        while not rospy.is_shutdown():
            self.wpCompletePub.publish(self.waypointCheck())
            self.rate.sleep()
        pass

if __name__ == "__main__":
    dc = droneCore()
    dc.run()
