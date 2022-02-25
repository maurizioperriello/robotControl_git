#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Dec 22 11:44:51 2021

@author: mauri
"""

########################################################################
#
#   CONTROLLER
#   Questa classe ha il compito di gestire i vari nodi di ROS, sia
#   Publisher che Subscriber, per poter comunicare con la simulazione
#   su CoppeliaSim
#
########################################################################

import rospy
from std_msgs.msg import Float64MultiArray, Int8MultiArray, Bool, Float64
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point
import numpy as np

"""
Di seguito la lista dei topic standard per il robot:

#Publisher Topic
robotPos_pubTop = 'joints_pose'
robotVel_pubTop = 'joints_velocity'
targetPos_pubTop = 'sphere_pose'
opPos_pubTop = 'op_pose'
opVel_pubTop = 'obstacle_op_velocity'
resetRobot_pubTop = 'reset_robot'
table_pubTopic = 'table_pose'

#Subscriber Topic
op_subs = 'obstacle_op_pos'
EEpos_subs = 'ee_pose'
EEvel_subs = 'ee_velocity'
targetPos_subs = 'target_pos'
robot_subs = 'joints_value'
spatialPosRobotJoints_subs = 'spatialPos_joints'
selfCollisionRobot_subs = 'collision_robot'
globalCollisionRobot_subs = 'global_robot_collision'
spatialPosFinalLinks_subs = 'spatialPos_finalLinks'
"""


class Controller:
    def __init__(self,nodeName, robotPos_pubTopic, robotVel_pubTopic, targetPos_pubTopic, opPos_pubTopic,
                 opVel_pubTopic, resetRobot_pubTopic, op_subs, EEpos_subs,
                 EEvel_subs, targetPos_subs, robot_subs, spatialPosRobotJoints_subs,
                 selfCollisionRobot_subs, globalCollisionRobot_subs,
                 spatialPosFinalLinks_subs, table_pubTopic):
        
        rospy.init_node(nodeName, anonymous=True) #inizializzazione del nodo
        
        #Publishers
        self.pub_robotControl_pos = rospy.Publisher(robotPos_pubTopic,
                                                    JointState, queue_size=10)
        self.pub_robotControl_vel = rospy.Publisher(robotVel_pubTopic,
                                                    Float64MultiArray,
                                                    queue_size=10)
        self.pub_targetControl_pos = rospy.Publisher(targetPos_pubTopic,
                                                     Float64MultiArray,
                                                     queue_size=10)
        self.pub_opControl_pos = rospy.Publisher(opPos_pubTopic,
                                                 Float64MultiArray,
                                                 queue_size=10)
        self.pub_opControl_vel = rospy.Publisher(opVel_pubTopic,
                                                 Float64MultiArray,
                                                 queue_size=10)
        self.pub_reset_robot = rospy.Publisher(resetRobot_pubTopic,
                                               Bool, queue_size=10)
        self.pub_table_pose = rospy.Publisher(table_pubTopic,
                                              Float64MultiArray, queue_size=10)
        
        
        self.rate = rospy.Rate(10)
        
        #Publisher messages
        self.reset_robot_msg = Bool()
        self.reset_robot_msg.data = True
        
        #self.start_pos_robot = [0, -1.57, 1.57, -6.16, 4.34, 2.36]        
        self.start_pos_robot = [0.0, -1.57, 1.57, -1.57, 4.71, 2.36]
        self.pos_robot_msg = JointState()
        self.pos_robot_msg.position = self.start_pos_robot
        
        self.start_vel_robot = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.vel_robot_msg = Float64MultiArray()
        self.vel_robot_msg.data = self.start_vel_robot
        
        self.pos_operator_msg = Float64MultiArray()
        
        self.vel_operator_msg = Float64MultiArray()
        
        self.start_position_target_msg = Float64MultiArray()
        
        self.table_pos_msg = Float64MultiArray()
        
        #self.operator_limit = [[-0.8, 0.8], [-0.8, 0.8], [0.1, 0.8]]
        
        #Subscribers
        self.sub_operator = rospy.Subscriber(op_subs, JointState,
                                             self.operator_pos_vel_callback)
        self.sub_EE_pos = rospy.Subscriber(EEpos_subs, Point,
                                             self.EE_pos_callback)
        self.sub_EE_vel = rospy.Subscriber(EEvel_subs, Point,
                                             self.EE_vel_callback)
        self.sub_target = rospy.Subscriber(targetPos_subs, JointState,
                                           self.target_pos_callback)
        self.sub_robot = rospy.Subscriber(robot_subs, JointState,
                                          self.robot_pos_vel_callback)
        self.sub_joints_spatialPos = rospy.Subscriber(spatialPosRobotJoints_subs,
                                                      Float64MultiArray,
                                                      self.joints_spatialPos_callback)
        self.sub_selfCollision_robot = rospy.Subscriber(selfCollisionRobot_subs,
                                                        Int8MultiArray,
                                                        self.robot_selfCollision_callback)
        self.sub_globalCollision_robot = rospy.Subscriber(globalCollisionRobot_subs,
                                                        Float64,
                                                        self.robot_globalCollision_callback)
        self.sub_finalLinks_spatialPos = rospy.Subscriber(spatialPosFinalLinks_subs,
                                                          Float64MultiArray,
                                                          self.finalLinks_spatialPos_callback)
        
        
        #Subscriber Data
        self.operator_pos = np.zeros(3)
        self.operator_vel = np.zeros(3)
    
        self.EE_pos = np.zeros(3)
        self.EE_vel = np.zeros(3)
        
        self.target_pos = np.zeros(3)
        
        self.robot_pos = np.zeros(6)
        self.robot_vel = np.zeros(6)
        
        self.joints_spatialPos = np.zeros(18)
        
        self.selfCollision_bool = False
        self.globalCollision_bool = False

        self.finalLinks_spatialPos = np.zeros(6)
        
        
        ###################
        #   Bill
        ###################
        
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
        self.spatialPose = [ [ 0 for _ in range(3) ]
                            for _ in range(8) ]
        #la velocità è uguale a quella impostata, perciò
        #si può recuperare direttamente dal dato impostato e 
        #non da CoppeliaSim
        
        
    #Publish functions
    def robot_vel_publish(self, vel):
        self.vel_robot_msg.data = vel
        self.pub_robotControl_vel.publish(self.vel_robot_msg)
        #self.rate.sleep()
        
    def robot_pos_publish(self, pos=[], reset=True):
        if reset:
            position = self.start_pos_robot
        else:
            position = pos
        self.pos_robot_msg.position = position
        self.pub_robotControl_pos.publish(self.pos_robot_msg)
        #self.rate.sleep()
        
    def target_pos_publish(self, pos):
        self.start_position_target_msg.data = pos
        self.pub_targetControl_pos.publish(self.start_position_target_msg)
        #self.rate.sleep()
        
    def operator_pos_publish(self, pos):
        self.pos_operator_msg.data = pos
        self.pub_opControl_pos.publish(self.pos_operator_msg)
        #self.rate.sleep()
        
    def operator_vel_publish(self, vel):
        self.vel_operator_msg.data = vel
        self.pub_opControl_vel.publish(self.vel_operator_msg)
        #self.rate.sleep()
    
    def robot_reset_publish(self):
        self.pub_reset_robot.publish(self.reset_robot_msg)
        #self.rate.sleep()    
        
    def table_publish(self, pos):
        self.table_pos_msg.data = pos
        self.pub_table_pose.publish(self.table_pos_msg)
        #self.rate.sleep()
    
        
    #Subscriber callback functions
    def operator_pos_vel_callback(self, msg):
        self.operator_pos = msg.position
        self.operator_vel = msg.velocity
        
    def EE_pos_callback(self, msg):
        self.EE_pos = [msg.x, msg.y, msg.z]
        
    def EE_vel_callback(self, msg):
        self.EE_vel = [msg.x, msg.y, msg.z]
    
    def target_pos_callback(self, msg):
        self.target_pos = msg.position
        
    def robot_pos_vel_callback(self, msg):
        self.robot_pos = msg.position
        self.robot_vel = msg.velocity
        
    def joints_spatialPos_callback(self, msg):
        self.joints_spatialPos = [ [ msg.data[j] for j in range(3*i, 3+3*i) ]
                  for i in range(6) ]
        
    def robot_selfCollision_callback(self, msg):
        collision_data = msg.data[0:4]
        self.selfCollision_bool = np.array(collision_data).any()
    
    def robot_globalCollision_callback(self, msg):
        self.globalCollision_bool = msg.data > 1
        
    def finalLinks_spatialPos_callback(self, msg):
        self.finalLinks_spatialPos = [ [ msg.data[j] for j in range(3*i, 3+3*i) ]
                  for i in range(2) ]
        
    #Getter functions (inutilizzate)
    def get_operator_pos_vel(self):
        return self.operator_pos, self.operator_vel
        
    def get_EE_pos_vel(self):
        self.sub_EE_vel()
        return self.EE_pos, self.EE_vel
        
    def get_target_position(self):
        return self.target_pos
        
    def get_robot_pos_vel(self):
        return self.robot_pos, self.robot_vel
        
    def get_joints_spatialPos(self):
        return self.joints_spatialPos
        
    def get_selfCollisionBool(self):
        return self.selfCollision_bool
        
    def get_globalCollisionBool(self):
        return self.globalCollision_bool
    
    def get_finalLinks_spatialPos(self):
        return self.finalLinks_spatialPos
    
    
    ###################
    #Bill stuff
    ###################
    
    #Publish functions
    def vel_publishFun(self, v):
        self.vel_msg.data = v
        self.vel_pub.publish(self.vel_msg)
        self.rate.sleep()    
    def reset_publishFun(self):
        self.reset_pub.publish(self.reset_msg)
        #self.rate.sleep()
       
    #Callback function    
    def spatialPose_callback(self, msg):
        self.sub_spatialPose = [ [ msg.data[j] for j in range(3*i, 3+3*i) ]
                                for i in range(8) ]
        