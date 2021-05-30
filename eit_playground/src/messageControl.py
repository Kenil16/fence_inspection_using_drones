#!/usr/bin/env python

import math
import utm
import rospy

import mavros.utils
import mavros.setpoint as mavSP
import mavros.command as mavCMD

import mavros_msgs.msg
import mavros_msgs.srv

from std_msgs.msg import (Bool, String, Int8)
from geometry_msgs.msg import (Quaternion)

from tf.transformations import quaternion_from_euler
from sensor_msgs.msg import NavSatFix

mavros.set_namespace('mavros')
onB_StateSub     = '/onboard/state'
onB_substateSub  = '/onboard/substate'

# Add Subscriber topics
loiterSub         = '/onboard/setpoint/loiter'
missionSub        = '/onboard/setpoint/mission'


class msgControl():
    def __init__(self):
        rospy.init_node('msgControl')
        self.rate = rospy.Rate(20)
        self.enable = False

        self.sysState = None
        self.sysSubState = None 
        self.MAVROS_State = mavros_msgs.msg.State()

        self.homePos = None
        #self.gotMission = False

        self.loiterMsg = None
        self.missionMsg = None
        #self.pylonNavMsg = None
        #self.inspectMsg = None
        #self.apAlignMsg = None
        #self.exampleGPSMsg = None
        #self.exampleATTIMsg = None 
        #self.lidarLandingATTI_msg = None


        self.setpointGPS    = mavSP.PoseStamped()
        self.setpointATTI   = mavros_msgs.msg.AttitudeTarget()

        self.curLocalPos    = mavSP.PoseStamped()

        self.setpointGPSPub = mavSP.get_pub_position_local(queue_size=1)
        self.setpointATTIPub = mavSP.get_pub_attitude_pose(queue_size=1)
        # self.setpointATTIPub = rospy.publisher('mavros/setpoint_raw/attitude', mavros_ms)

        # root_framework Subs
        rospy.Subscriber(onB_StateSub, String, self.onStateChange)
        rospy.Subscriber(onB_substateSub, String, self.onsubStateChange)

        rospy.Subscriber(mavros.get_topic('state'),
                         mavros_msgs.msg.State, self._cb_uavState)
        rospy.Subscriber('/onboard/enableMH', Bool, self.handlerEnable)

        self._mavrosHandshake()

        rospy.Subscriber(mavros.get_topic('local_position', 'pose'),
                         mavSP.PoseStamped,
                         self._cb_localPosUpdate)
        rospy.Subscriber(mavros.get_topic('home_position', 'home'),
                         mavros_msgs.msg.HomePosition,
                         self._cb_onHomeUpdate)
        # pilot subs
        rospy.Subscriber(loiterSub, mavSP.PoseStamped, self.pilot_loiterMsg)
        
        rospy.Subscriber(missionSub, mavSP.PoseStamped, self.pilot_missionMsg)
        
        #rospy.Subscriber(exampleSubGPS, mavSP.PoseStamped, self.pilot_exampleGPSMsg)
        #rospy.Subscriber(exampleSubAtti, mavros_msgs.msg.AttitudeTarget, self.pilot_exampleATTIMsg)
        #rospy.Subscriber(lidarLandingAtti, mavros_msgs.msg.AttitudeTarget, self.pilot_lidarlandingMsg)

        self.setMode = rospy.ServiceProxy('/mavros/set_mode', mavros_msgs.srv.SetMode)

    def _mavrosHandshake(self):
        rospy.loginfo('MessageHandler: Waiting for MAVROS Connection.')
        i = 0
        time = rospy.Time.now()
        for i in range(0, 3):
            print'.',
            if self.MAVROS_State.connected:
                rospy.loginfo("MessageHandler: MAVROS Connected!")
                break
            rospy.sleep(1)

        if not self.MAVROS_State.connected:
            errorMsg = "MAVROS not connected, will try again in 30 Seconds."
            rospy.logfatal(errorMsg)
            for i in range(1, 30):
                rospy.sleep(1)
                if self.MAVROS_State.connected:
                    rospy.loginfo("MessageHandler: MAVROS Connected!")
                    break

    def handlerEnable(self, msg):
        if msg.data == True:
            rospy.loginfo("MessageHandler: Enabled")
            self.enable = True
        if msg.data == False:
            if self.enable:
                rospy.loginfo("MessageHandler: Disabled")
            self.enable = False

    def _pubMsg(self, msg, topic):
        msg.header.stamp = rospy.Time.now()
        topic.publish(msg)
        self.rate.sleep()

    def onStateChange(self, msg):
        if msg.data == 'idle':
            self.enable = False
            rospy.loginfo('MessageHandler: Disabled')
        else:
            self.sysState = msg.data

    def onsubStateChange(self,msg):
            self.sysSubState = msg

    def _cb_localPosUpdate(self, msg):
        self.curLocalPos = msg

    def _cb_onHomeUpdate(self, msg):
        if self.homePos == None:
            pass
        self.homePos = msg

    def _cb_uavState(self, msg):
        self.MAVROS_State = msg
        pass

    def onPositionChange(self, msg):
        self.curLocalPos = msg

    # pilotMessage Handlers

    #def pilot_exampleGPSMsg(self,msg):
        #self.exampleGPSMsg = msg

    #def pilot_exampleATTIMsg(self,msg):
        #print("gotAttiMsg")
        #self.exampleATTIMsg = msg

    def pilot_loiterMsg(self, msg):
        self.loiterMsg = msg

    def pilot_missionMsg(self, msg):
        self.missionMsg = msg

    #def pilot_apAlignMsg(self, msg):
        #self.apAlignMsg = msg

    #def pilot_pylonNavMsg(self, msg):
        # print(msg)
        #x, y, z = self.gpsToLocal(msg)
        #orient = math.atan2(y, x)
        #if x > 0:
            #orientation = Quaternion(
                #*quaternion_from_euler(0, 0, orient+math.pi))
        #else:
            #orientation = Quaternion(*quaternion_from_euler(0, 0, -orient))
        #tmpSP = mavSP.PoseStamped()
        #tmpSP.pose.position.x = y
        #tmpSP.pose.position.y = x
        #tmpSP.pose.position.z = z

        #tmpSP.pose.orientation = orientation

        #self.pylonNavMsg = tmpSP

        #self.sysState = 'mission'
        #self.gotMission = True

    #def pilot_lidarlandingMsg(self,msg):
        #self.lidarLandingATTI_msg = msg

    #def _onInspectPosUpdate(self, msg):
        #self.inspectMsg = msg
        #pass
    


    def get_pilotMsg(self):
        outwardMsg = None
        if self.sysState == 'loiter':
            outwardMsg = self.loiterMsg
            #self.loiterMsg = None

        if self.sysState == 'mission':
            outwardMsg = self.missionMsg
            #self.missionMsg = None

        return outwardMsg

    def publishPilotMsg(self):
        outMsg = self.get_pilotMsg()

        if self.homePos == None:
            rospy.logwarn("MessageHandler: unable to publish, no home position")

        if outMsg == None:
            rospy.logfatal_once(
                "MessageHandler: Received no message. Has a pilot crashed?")
            self.setMode(0, "AUTO.LOITER")
        else:
            if (outMsg._type == "geometry_msgs/PoseStamped"):
                self._pubMsg(outMsg, self.setpointGPSPub)

            
# Positioning calcuations
    def calcDist(self, utmPosA, utmPosB):
        dist = -1
        deltaEast = utmPosA[0]-utmPosB[0]
        deltaNorth = utmPosA[1]-utmPosB[1]
        if deltaNorth != 0:
            dist = math.sqrt(deltaNorth**2 + deltaEast**2)
        return deltaNorth, deltaEast, dist

    def gpsToLocal(self, gpsPos):
        utmPos = utm.from_latlon(gpsPos.latitude, gpsPos.longitude)

        utmHome = utm.from_latlon(
            self.homePos.geo.latitude, self.homePos.geo.longitude)

        deltaNorth, deltaEast, _ = self.calcDist(utmPos, utmHome)
        deltaAlt = gpsPos.altitude
        # print ("Local Point: %.4f, %.4f, %.2f" % (deltaNorth, deltaEast, gpsPos.altitude))
        return deltaNorth, deltaEast, deltaAlt

    def run(self):
        while not rospy.is_shutdown():
            if self.enable:
                self.publishPilotMsg()
                # self._pubMsg(self.setpointGPS, self.setpointGPSPub)

            self.rate.sleep()


if __name__ == "__main__":
    LP = msgControl()
    LP.run()
