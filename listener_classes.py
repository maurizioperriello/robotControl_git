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
from geometry_msgs.msg import Point, Pose
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
    def __init__(self,nodeName='robotControl', robotPos_pubTopic='joints_pose',
                 robotVel_pubTopic='joints_velocity',
                 targetPos_pubTopic='sphere_pose',
                 opPos_pubTopic='op_pose',
                 opVel_pubTopic='obstacle_op_velocity',
                 resetRobot_pubTopic='reset_robot',
                 op_subs='obstacle_op_pos', EEpos_subs='ee_pose',
                 EEvel_subs='ee_velocity', targetPos_subs='target_pos',
                 robot_subs='joints_value',
                 spatialPosRobotJoints_subs='spatialPos_joints',
                 selfCollisionRobot_subs='collision_robot',
                 globalCollisionRobot_subs='global_robot_collision',
                 spatialPosFinalLinks_subs='spatialPos_finalLinks',
                 table_pubTopic='table_pose',
                 redCube_pubTopic='redCube_posDef', greenCube_pubTopic='greenCube_posDef',
                 blueCube_pubTopic='blueCube_posDef', yellowCube_pubTopic='yellowCube_posDef',
                 greyCube_pubTopic='greyCube_posDef', redCube_subs='redCube_target_pos',
                 greenCube_subs='greenCube_target_pos', blueCube_subs='blueCube_target_pos',
                 yellowCube_subs='yellowCube_target_pos', greyCube_subs='greyCube_target_pos',
                 ee_pose_subs='end_effector_pose'):
        
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
        self.pub_redCube_pos = rospy.Publisher(redCube_pubTopic,
                                               Float64MultiArray, queue_size=10)
        self.pub_greenCube_pos = rospy.Publisher(greenCube_pubTopic,
                                                 Float64MultiArray, queue_size=10)
        self.pub_blueCube_pos = rospy.Publisher(blueCube_pubTopic,
                                                Float64MultiArray, queue_size=10)
        self.pub_yellowCube_pos = rospy.Publisher(yellowCube_pubTopic,
                                                  Float64MultiArray, queue_size=10)
        self.pub_greyCube_pos = rospy.Publisher(greyCube_pubTopic,
                                                Float64MultiArray, queue_size=10)
        
        
        self.rate = rospy.Rate(100)
        
        #Publisher messages
        self.reset_robot_msg = Bool()
        self.reset_robot_msg.data = True
        
        #self.start_pos_robot = [0, -1.57, 1.57, -6.16, 4.34, 2.36]        
        self.start_pos_robot = [0.0, -1.57, 1.57, -1.57, 4.71, 2.36]
        #self.start_pos_robot = [0.0, -1.57, 1.57, -1.56, -1.52, 0.0] #ur10e definitivo        
        
        self.pos_robot_msg = JointState()
        self.pos_robot_msg.position = self.start_pos_robot
        
        self.start_vel_robot = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.vel_robot_msg = Float64MultiArray()
        self.vel_robot_msg.data = self.start_vel_robot
        
        self.pos_operator_msg = Float64MultiArray()
        
        self.vel_operator_msg = Float64MultiArray()
        
        self.start_position_target_msg = Float64MultiArray()
        
        self.table_pos_msg = Float64MultiArray()
        
        #cube
        self.redCube_pos_msg = Float64MultiArray()
        self.greenCube_pos_msg = Float64MultiArray()
        self.blueCube_pos_msg = Float64MultiArray()
        self.yellowCube_pos_msg = Float64MultiArray()
        self.greyCube_pos_msg = Float64MultiArray()
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
        self.sub_redCube_pos = rospy.Subscriber(redCube_subs,
                                                JointState,
                                                self.redCubePos_callback)
        self.sub_greenCube_pos = rospy.Subscriber(greenCube_subs,
                                                  JointState,
                                                  self.greenCubePos_callback)
        self.sub_blueCube_pos = rospy.Subscriber(blueCube_subs,
                                                 JointState,
                                                 self.blueCubePos_callback)
        self.sub_yellowCube_pos = rospy.Subscriber(yellowCube_subs,
                                                   JointState,
                                                   self.yellowCubePos_callback)
        self.sub_greyCube_pos = rospy.Subscriber(greyCube_subs,
                                                 JointState,
                                                 self.greyCubePos_callback)
        
        self.sub_ee_orientation = rospy.Subscriber(ee_pose_subs,
                                                   Pose,
                                                   self.eePose_callback)
        
        #Subscriber Data
        self.operator_pos = np.zeros(3)
        self.operator_vel = np.zeros(3)
    
        self.EE_pos = np.zeros(3)
        self.EE_vel = np.zeros(3)
        self.EE_orientation = np.zeros(4)
        
        self.target_pos = np.zeros(3)
        
        self.robot_pos = np.zeros(6)
        self.robot_vel = np.zeros(6)
        self.robot_acc = np.zeros(6)
        
        self.joints_spatialPos = np.zeros(18)
        
        self.selfCollision_bool = False
        self.globalCollision_bool = False

        self.finalLinks_spatialPos = np.zeros(6)
        
        self.redCube_pos = np.zeros(3)
        self.greenCube_pos = np.zeros(3)
        self.blueCube_pos = np.zeros(3)
        self.yellowCube_pos = np.zeros(3)
        self.greyCube_pos = np.zeros(3)
        
        
        ###################
        #   Bill
        ###################
        
        #Publishers
        """
        self.vel_pub = rospy.Publisher('bill_velocity', Float64MultiArray,
                                       queue_size=10)
        self.reset_pub = rospy.Publisher('bill_reset', Bool,
                                         queue_size=10)
        """
        self.billHand_pub = rospy.Publisher('bill_handPose', Float64MultiArray,
                                            queue_size=10)
        
        #Publisher Msg
        """
        self.vel_msg = Float64MultiArray()
        self.vel_msg.data = np.zeros(10)
        
        self.reset_msg = Bool()
        self.reset_msg.data = True
        """
        
        self.billHand_msg = Float64MultiArray()
        self.billHand_msg.data = np.zeros(4)
        
        #Subscriber
        """
        self.sub_spatialPose = rospy.Subscriber('bill_spatialPose',
                                                Float64MultiArray,
                                                self.spatialPose_callback)
        """
        self.billLimb_position_sub = rospy.Subscriber('bill_limbPosition',
                                                  Float64MultiArray,
                                                  self.billLimbPos_callback)
        
        #Subscriber Data
        """
        self.spatialPose = [ [ 0 for _ in range(3) ]
                            for _ in range(8) ]
        #la velocità è uguale a quella impostata, perciò
        #si può recuperare direttamente dal dato impostato e 
        #non da CoppeliaSim
        """
        self.billLimb_spatialPos = [ [ 0 for _ in range(3) ]
                                    for _ in range(8) ]
        self.billLimb_spatialVel = [ [ 0 for _ in range(3) ]
                                    for _ in range(8) ]
        self.start_billLimb_vect = [ [ 0 for _ in range(3) ]
                                    for _ in range(8) ]
        
        
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
        
    def redCube_publish(self, pos):
        self.redCube_pos_msg.data = pos
        self.pub_redCube_pos.publish(self.redCube_pos_msg)
        self.rate.sleep()
        
    def greenCube_publish(self, pos):
        self.greenCube_pos_msg.data = pos
        self.pub_greenCube_pos.publish(self.greenCube_pos_msg)
        self.rate.sleep()
        
    def blueCube_publish(self, pos):
        self.blueCube_pos_msg.data = pos
        self.pub_blueCube_pos.publish(self.blueCube_pos_msg)
        self.rate.sleep()
        
    def yellowCube_publish(self, pos):
        self.yellowCube_pos_msg.data = pos
        self.pub_yellowCube_pos.publish(self.yellowCube_pos_msg)
        self.rate.sleep()
        
    def greyCube_publish(self, pos):
        self.greyCube_pos_msg.data = pos
        self.pub_greyCube_pos.publish(self.greyCube_pos_msg)
        self.rate.sleep()
    
        
    #Subscriber callback functions
    def eePose_callback(self, msg):
        self.EE_orientation = [msg.orientation.x, msg.orientation.y,
                               msg.orientation.z, msg.orientation.w]
    
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
        self.robot_acc = msg.effort
        
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
        
    def redCubePos_callback(self, msg):
        self.redCube_pos = msg.position
        
    def greenCubePos_callback(self, msg):
        self.greenCube_pos = msg.position
        
    def blueCubePos_callback(self, msg):
        self.blueCube_pos = msg.position
        
    def yellowCubePos_callback(self, msg):
        self.yellowCube_pos = msg.position
        
    def greyCubePos_callback(self, msg):
        self.greyCube_pos = msg.position
        
    #Getter functions (inutilizzate)
    def get_operator_pos_vel(self):
        return self.operator_pos, self.operator_vel
        
    def get_EE_pos_vel(self):
        self.sub_EE_vel()
        return self.EE_pos, self.EE_vel
        
    def get_target_position(self):
        return self.target_pos
        
    def get_robot_pos_vel_acc(self):
        return self.robot_pos, self.robot_vel, self.robot_acc
        
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
    """
    #Publish functions
    def vel_publishFun(self, v):
        self.vel_msg.data = v
        self.vel_pub.publish(self.vel_msg)
        #self.rate.sleep()    
    def reset_publishFun(self):
        self.reset_pub.publish(self.reset_msg)
        #self.rate.sleep()
       
    #Callback function    
    def spatialPose_callback(self, msg):
        self.sub_spatialPose = [ [ msg.data[j] for j in range(3*i, 3+3*i) ]
                                for i in range(8) ]
    """
    #Publish functions  
    def billHandPos_publishFun(self, pos):
        self.billHand_msg.data = pos
        self.billHand_pub.publish(self.billHand_msg)
        #self.rate.sleep()
        
    #Callback function
    def billLimbPos_callback(self, msg):
        check = False
        old_spatialPos = self.start_billLimb_vect
        if(self.billLimb_spatialPos != self.start_billLimb_vect):
            check = True
            old_spatialPos = self.billLimb_spatialPos
        
        self.billLimb_spatialPos = [ [ msg.data[j] for j in range(3*i, 3+3*i) ]
                                    for i in range(8) ]
        if check:
            self.billLimb_spatialVel = [ [ (self.billLimb_spatialPos[i][j]-old_spatialPos[i][j])/0.05
                                          for j in range(3) ] 
                                            for i in range(8) ]
        
        