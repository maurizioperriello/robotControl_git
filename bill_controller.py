#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Feb 16 10:41:14 2022

@author: mauri
"""

import rospy
from std_msgs.msg import Float64MultiArray, Bool
import numpy as np

class Bill:
    def __init__(self):
        rospy.init_node('bill_ctrl', anonymous=True)
        
        self.rate = rospy.Rate(10)
        
        #Publishers
        self.vel_pub = rospy.Publisher('bill_velocity', Float64MultiArray,
                                       queue_size=10)
        self.reset_pub = rospy.Publisher('bill_reset', Bool,
                                         queue_size=10)
        
        #Publisher Msg
        self.vel_msg = Float64MultiArray()
        self.vel_msg.data = np.zeros(10)
        
        self.reset_msg = Bool()
        self.reset_msg.data = True
        
        #Subscriber
        self.sub_spatialPose = rospy.Subscriber('bill_spatialPose',
                                                Float64MultiArray,
                                                self.spatialPose_callback)
        
        #Subscriber Data
        self.spatialPose = np.zeros(24)
        #la velocità è uguale a quella impostata, perciò
        #si può recuperare direttamente dal dato impostato e 
        #non da CoppeliaSim
        
    def spatialPose_callback(self, msg):
        self.sub_spatialPose = [ [ msg.data[j] for j in range(3*i, 3+3*i) ]
                                for i in range(8) ]
        
        
        