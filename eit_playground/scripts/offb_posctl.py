#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
offboard_ctrl.py: Controlling the setpoints

"""

###############################################
# Standard Imports                            #
###############################################
import time
import threading
from math import *
import numpy as np

###############################################
# ROS Imports                                 #
###############################################
import rospy
import rospkg

###############################################
# ROS Topic messages                          #
###############################################
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import NavSatFix
from mavros_msgs.msg import State


###############################################
# ROS Service messages                        #
###############################################
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL
from std_srvs.srv import Empty, EmptyRequest, EmptyResponse

###############################################
# Offboad Control class                       #
###############################################
class OffboardControl:
    def __init__(self, *args):
        self.current_state = State()
        #self.curr_global_pos = NavSatFix()

        rospy.init_node('offboard_ctrl')

        self.offb_set_mode = SetMode

        self.prev_state = self.current_state

        # Publishers
        self.local_pos_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)

        # Subscribers
        self.state_sub = rospy.Subscriber('/mavros/state', State, self.cb_state)
        self.sub_target = rospy.Subscriber('/mavros/offbctrl/target', PoseStamped, self.cb_target)

        # Services
        self.arming_client = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.takeoff_client = rospy.ServiceProxy('/mavros/cmd/takeoff', CommandTOL)
        self.set_mode_client = rospy.ServiceProxy('/mavros/set_mode', SetMode)

        ## Create services
        self.setpoint_controller_server()

        # Init msgs
        self.target = PoseStamped()
        self.target.pose.position.x = 0
        self.target.pose.position.y = 0
        self.target.pose.position.z = 2

        self.last_request = rospy.get_rostime()
        self.state = "INIT"

        self.rate = rospy.Rate(20.0) # MUST be more then 2Hz

        self.t_run = threading.Thread(target=self.navigate)
        self.t_run.start()
        print(">> SetPoint controller is running (Thread)")

        # Spin until the node is stopped

        time.sleep(5)
        tmp = Empty()
        self.switch2offboard(tmp)

        rospy.spin()

    """
    Callbacks
    * cb_state
    * cb_target
    """
    def cb_state(self,state):
        self.current_state = state

    def cb_target(self,data):
        self.set_target(data)

    """
    Services
    * s_arm:
    * s_stop:
    * s_s2o:
    * s_circle:
    """
    def setpoint_controller_server(self):
        s_arm = rospy.Service('setpoint_controller/arm', Empty, self.arm)
        s_stop = rospy.Service('setpoint_controller/stop', Empty, self.stop)
        s_s2o = rospy.Service('setpoint_controller/switch2offboard', Empty, self.switch2offboard)
        s_circle = rospy.Service('setpoint_controller/circle', Empty, self.start_circle)

        print("The SetPoint Controller is ready")

    """
    State
    * set_state:
    * get_state:
    """
    def set_state(self, data):
        self.state = data
        print("New State: {}".format(data))

    def get_state(self):
        return self.state

    """
    Target position
    * set_target:
    """
    def set_target(self, data):
        self.target = data

    def set_target_xyz(self,x,y,z,delay):

        if(delay > 0.1):
            print(">> New setpoint: {} {} {}".format(x,y,z))

        self.target.pose.position.x = x
        self.target.pose.position.y = y
        self.target.pose.position.z = z

        time.sleep(delay)

    def get_target(self):
        return self.target

    """
    Commands
    * switch2offboard:
    * arm:
    * navigate:
    * start_circle:
    * cicle:
    * stop: 
    """
    def switch2offboard(self,r):
        print(">> Starting OFFBOARD mode")

        last_request = rospy.get_rostime()
        while self.current_state.mode != "OFFBOARD":
            now = rospy.get_rostime()
            if(now - last_request > rospy.Duration(5)):
                print("Trying: OFFBOARD mode")
                self.set_mode_client(base_mode=0, custom_mode="OFFBOARD")
                last_request = now

        tmp = Empty()
        self.arm(tmp)

        return {}

    def arm(self,r):
        print(">> Arming...")
        last_request = rospy.get_rostime()
        while not self.current_state.armed:
            now = rospy.get_rostime()
            if(now - last_request > rospy.Duration(5.)):
               self.arming_client(True)
               last_request = now

        return {}

    def navigate(self):
        self.set_state("RUNNING")
        while not rospy.is_shutdown():
            self.target.header.frame_id = "base_footprint"
            self.target.header.stamp = rospy.Time.now()

            self.local_pos_pub.publish(self.target)
            self.rate.sleep()

        print(">> Navigation thread has stopped...")
        self.set_state("STOPPED")

    def start_circle(self, r):
        self.t_circle = threading.Thread(target=self.circle)
        self.t_circle.start()
        print(">> Starting circle (Thread)")

        return {}

    def circle(self):
        sides = 360
        radius = 1
        i = 0
        delay = 0.5

        if(self.state != "RUNNING"):
            print(">> SetPoint controller is not running...")
        else:
            self.set_state("CIRCLE")

            while self.state == "CIRCLE":
                x = radius * cos(i*2*pi/sides)
                y = radius * sin(i*2*pi/sides)
                z = self.target.pose.position.z

                self.set_target_xyz(x,y,z,delay)

                i = i + 1

                # Reset counter
                if(i > 360):
                    i = 0

            self.set_state("RUNNING")

    """
    Stop
    """
    def stop(self,r):
        self.set_state("STOP")
        return {}

if __name__ == '__main__':
    SPC = OffboardControl()
