#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue May 17 11:41:38 2022

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
from geometry_msgs.msg import Point, PoseStamped, TransformStamped
from tf2_msgs.msg import TFMessage
import numpy as np
from utils import HTrans, compute_ur_jacobian, rotationMatrixToEulerAngles
from math import pi

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
    def __init__(self,nodeName='robotControl',
                 robotVel_pubTopic='joint_group_vel_controller/command',
                 robot_subs='joint_states',
                 tf_subs='/tf',
                 operator_head_subs='vrpn_client_node/left_wrist/pose',
                 operator_rightHand_subs='vrpn_client_node/left_wrist/pose',
                 operator_leftHand_subs='vrpn_client_node/left_wrist/pose',
                 targetPos_subs='vrpn_client_node/left_wrist/pose',
                 redCube_subs='target_pos',
                 greenCube_subs='target_pos',
                 blueCube_subs='target_pos',
                 yellowCube_subs='target_pos',
                 greyCube_subs='target_pos'):
        
        rospy.init_node(nodeName, anonymous=True) #inizializzazione del nodo
        
        self.old_time = 0.0
        
        self.dt = 0.05 #durata step temporale
        #distanza base_robot da origine sistema di riferimento di CoppeliaSim
        self.t_vect_EE = [-0.6, 0.0, 0.0] #da sommare alla posizione dell'EE
        #distanza tra O coppeliaSim e O optirack
        self.t_vect_optitrack = [-1.11, 0.0, 0.0] #[-0.6, 0.51, 0.0] #old_config
        #da sommare alle posizioni in arrivo dall'optitrack
        #-90 deg in rad per cambiare il sistema di riferimento del robot,
        #dato che in lab è ruotato di +90 deg rispetto alla simulazione
        self.th90rad = -90*2*pi/360
        
        #matrice di rotazione per passare da sistema di riferimento dell'
        #optitrack a quello di coppeliaSim (+90deg)
        self.R_mat_optitrack = np.array([[0, -1, 0],
                                         [1, 0, 0],
                                         [0, 0, 1]])
        
        #matrice di rotazione per passare dal sistema di riferimento
        #della base del robot a quello di CoppeliaSim
        self.R_mat_ur10e = np.array([[0, -1, 0],
                                     [1, 0, 0],
                                     [0, 0, 1]])
        
        self.dz = 0.1 #distanza di sicurezza dall'oggetto lungo l'asse z
        
        #Publishers
        self.pub_robotControl_vel = rospy.Publisher(robotVel_pubTopic,
                                                    Float64MultiArray,
                                                    queue_size=10)
        
        self.rate = rospy.Rate(100)
        
        #self.start_pos_robot = [0, -1.57, 1.57, -6.16, 4.34, 2.36]        
        self.start_pos_robot = [0.0, -1.57, 1.57, -1.57, 4.71, 2.36]
        #self.start_pos_robot = [0.0, -1.57, 1.57, -1.56, -1.52, 0.0] #ur10e definitivo        
        
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
        
        #Subscribers
        self.sub_robot = rospy.Subscriber(robot_subs, JointState,
                                          self.robot_pos_vel_callback)
        self.sub_tf = rospy.Subscriber(tf_subs, TFMessage,
                                       self.tf_callback)
        self.sub_operatorHead_pos = rospy.Subscriber(operator_head_subs,
                                                     PoseStamped,
                                                     self.opHead_callback)
        self.sub_operatorRightHand_pos = rospy.Subscriber(operator_head_subs,
                                                     PoseStamped,
                                                     self.opRXh_callback)
        self.sub_operatorLeftHand_pos = rospy.Subscriber(operator_head_subs,
                                                     PoseStamped,
                                                     self.opLXh_callback)
        #Target Objects        
        self.sub_sphere_pos = rospy.Subscriber(targetPos_subs,
                                                PoseStamped,
                                                self.target_pos_callback)        
        self.sub_redCube_pos = rospy.Subscriber(redCube_subs,
                                                Float64MultiArray,
                                                self.redCubePos_callback)
        self.sub_greenCube_pos = rospy.Subscriber(greenCube_subs,
                                                  Float64MultiArray,
                                                  self.greenCubePos_callback)
        self.sub_blueCube_pos = rospy.Subscriber(blueCube_subs,
                                                 Float64MultiArray,
                                                 self.blueCubePos_callback)
        self.sub_yellowCube_pos = rospy.Subscriber(yellowCube_subs,
                                                   Float64MultiArray,
                                                   self.yellowCubePos_callback)
        self.sub_greyCube_pos = rospy.Subscriber(greyCube_subs,
                                                 Float64MultiArray,
                                                 self.greyCubePos_callback)
        
        #Subscriber Data
        
        self.opHead_pos = np.zeros(3)
        self.opRightHand_pos = np.zeros(3)        
        self.opLeftHand_pos = np.zeros(3)
        self.opHead_vel = np.zeros(3)
        self.opRightHand_vel = np.zeros(3)        
        self.opLeftHand_vel = np.zeros(3)
        
        self.tf_translation = np.zeros(3)
        self.tf_orientation = np.zeros(4)
    
        self.EE_oldPos = np.zeros(3)
        self.EE_pos = np.zeros(3)
        self.EE_vel = np.zeros(3)
        self.EE_orientation = np.zeros(3)
   
        self.robot_th = np.zeros(6)
        self.robot_vel = np.zeros(6)
        self.robot_acc = np.zeros(6)
        
        self.target_pos = np.zeros(3)
        self.redCube_pos = np.zeros(3)
        self.greenCube_pos = np.zeros(3)
        self.blueCube_pos = np.zeros(3)
        self.yellowCube_pos = np.zeros(3)
        self.greyCube_pos = np.zeros(3)
        
        
    #Publish functions
    def robot_vel_publish(self, vel):
        self.vel_robot_msg.data = vel
        self.pub_robotControl_vel.publish(self.vel_robot_msg)
        #self.rate.sleep()
    
        
    #Subscriber callback functions
    def tf_callback(self, msg):
        #data = msg.transforms[0].transform
        data = msg.transforms[-1].transform
        t_tmp = [ data.translation.x, data.translation.y, data.translation.z ]
        t_tmp = np.matmul(self.R_mat_ur10e, t_tmp)
        self.tf_translation = [ t_tmp[i]+self.t_vect_EE[i] for i in range(3) ]
        self.tf_orientation = [ data.rotation.x, data.rotation.y,
                                data.rotation.z, data.rotation.w ]
    
    def operator_pos_vel_callback(self, msg):
        self.operator_pos = msg.position
        self.operator_vel = msg.velocity
           
    def robot_pos_vel_callback(self, msg):
        #Si scambiano i valori del primo e del terzo giunto poiché
        #nel messaggio sono scambiati
        
        #Time
        time = rospy.get_time()
        delta_t = time - self.old_time
        self.old_time = time

        th_tmp_tuple = msg.position
        th_tmp = list(th_tmp_tuple)
        th0 = th_tmp[0]
        th_tmp[0] = th_tmp[2] #+ self.th90rad
        th_tmp[2] = th0        
        self.robot_vel = msg.velocity
        self.robot_acc = msg.effort
        vel = [ self.robot_vel[i]/0.1 for i in range(6) ]
        
        th = np.matrix([ [th_tmp[i]] for i in range(6) ])
        T_EE = HTrans(th, 0)
        J = compute_ur_jacobian(th_tmp)
        EE_pos_tmp = np.matmul(self.R_mat_ur10e, 
                               [ T_EE[i,3] for i in range(3) ])
        self.EE_oldPos = np.array(self.EE_pos).copy()
        self.EE_pos = [ EE_pos_tmp[i]+self.t_vect_EE[i] for i in range(3) ]
        EE_vel_tmp = np.dot(J, vel)[0:3] #TODO: verificare!!!
        self.EE_vel = np.matmul(self.R_mat_ur10e, EE_vel_tmp)
        self.EE_orientation = np.matmul(self.R_mat_ur10e,
                                        rotationMatrixToEulerAngles(T_EE[0:3,0:3]))
        
        checks = [ self.EE_oldPos[i]!=0.0 for i in range(3) ]
        if np.array(checks).any():
            self.EE_vel = [ (self.EE_pos[i]-self.EE_oldPos[i])/delta_t
                               for i in range(3) ]

        th_tmp[0] = th_tmp[2] + self.th90rad       
        self.robot_th = th_tmp
        
    def opHead_callback(self, msg):
        data = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
        check = False
        old_spatialPos = np.zeros(3)
        if(np.array([ self.opLeftHand_pos[i]!=0.0 for i in range(3) ]).any()):
            check = True
            old_spatialPos = self.opHead_pos
        self.opHead_pos = [ data[i]+self.t_vect_optitrack[i]
                           for i in range(3) ]
        if check:
            self.opHead_vel = [ (self.opHead_pos[i]-old_spatialPos[i])/self.dt
                               for i in range(3) ]
            
    def opRXh_callback(self, msg):
        data = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
        check = False
        old_spatialPos = np.zeros(3)
        if(np.array([ self.opLeftHand_pos[i]!=0.0 for i in range(3) ]).any()):
            check = True
            old_spatialPos = self.opRightHand_pos
        self.opRightHand_pos = [ data[i]+self.t_vect_optitrack[i]
                                for i in range(3) ]
        if check:
            self.opRightHand_vel = [ (self.opRightHand_pos[i]-old_spatialPos[i])/self.dt
                               for i in range(3) ] 
    
    def opLXh_callback(self, msg):
        data = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
        check = False
        old_spatialPos = np.zeros(3)
        if(np.array([ self.opLeftHand_pos[i]!=0.0 for i in range(3) ]).any()):
            check = True
            old_spatialPos = self.opLeftHand_pos
        self.opLeftHand_pos = [ data[i]+self.t_vect_optitrack[i]
                                for i in range(3) ]
        if check:
            self.opLeftHand_vel = [ (self.opLeftHand_pos[i]-old_spatialPos[i])/self.dt
                               for i in range(3) ] 
        
    
    
    def target_pos_callback(self, msg):
        data = [msg.pose.position.x,
                msg.pose.position.y, 
                msg.pose.position.z + self.dz]
        data = np.matmul(self.R_mat_optitrack, data)
        self.target_pos = [ data[i]+self.t_vect_optitrack[i] 
                           for i in range(3) ]
        
    def redCubePos_callback(self, msg):
        self.redCube_pos = [ msg.data[i]+self.t_vect_optitrack[i] 
                            for i in range(3) ]
        
    def greenCubePos_callback(self, msg):
        self.greenCube_pos = [ msg.data[i]+self.t_vect_optitrack[i] 
                              for i in range(3) ]
        
    def blueCubePos_callback(self, msg):
        self.blueCube_pos = [ msg.data[i]+self.t_vect_optitrack[i] 
                             for i in range(3) ]
        
    def yellowCubePos_callback(self, msg):
        self.yellowCube_pos = [ msg.data[i]+self.t_vect_optitrack[i] 
                               for i in range(3) ]
        
    def greyCubePos_callback(self, msg):
        self.greyCube_pos = [ msg.data[i]+self.t_vect_optitrack[i] 
                             for i in range(3) ]
        
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

        