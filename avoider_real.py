#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat May 28 19:57:48 2022

@author: mauri
"""

import rospy
from controller_real_class import Controller
import numpy as np
from ddpg_classes import Agent
from utils import compute_ur_jacobian
from math import pi, copysign
from scipy.spatial.transform import Rotation as R


def observe(ctrl, action, theta_target):
    R_mat_ur10e = np.array([[0, -1, 0],
                            [1, 0, 0],
                            [0, 0, 1]])
    th90rad = -90*2*pi/360
    
    #theta_joints = np.array(ctrl.robot_th).copy()
    EE_pos = ctrl.tf_translation.copy()
    
    #Per passare dalla posizione del TCP a quella dell'EE,
    #come in simulazione
    T_d = np.matrix(np.identity(4), copy=False)
    T_d[2,3] = 0.195
    EE_pos.append(1.0)
    EE_pos = np.dot(T_d, EE_pos)
    EE_pos = [ EE_pos[0,i] for i in range(3) ]
    
    th = ctrl.robot_th.copy()
    th[0] = th[0] - th90rad
    J = compute_ur_jacobian(th)
    EE_vel = np.dot(J, action)[0:3]
    EE_vel = np.matmul(R_mat_ur10e, EE_vel)
    billLimbs = [ list(np.array(ctrl.opHead_pos).copy()), 
                  list(np.array(ctrl.opRightHand_pos).copy()),
                  list(np.array(ctrl.opLeftHand_pos).copy()) ]
    billLimbs_vel = [ list(np.array(ctrl.opHead_vel).copy()), 
                      list(np.array(ctrl.opRightHand_vel).copy()),
                      list(np.array(ctrl.opLeftHand_vel).copy()) ]
    th[0] = th[0] + th90rad
    
    #print(f'pos = {billLimbs}', f'vel = {billLimbs_vel}', sep='\n')
    #print(f'th_target = {theta_target}', f'th = {theta_joints}', sep='\n')
    #0.0, -1.57, 1.57
    
    obs = []
    
    for l in range(3):
        for k in range(3):
            obs.append(billLimbs[l][k])
        for h in range(3):
            obs.append(billLimbs_vel[l][h])                
    for i in range(3): 
        obs.append(theta_target[i])            
    for i in range(3):   
        obs.append(theta_target[i]-th[i])          
    for i in range(3):
        obs.append(EE_pos[i])               
    for i in range(3):
        obs.append(EE_vel[i])       
    for i in range(3):
        obs.append(th[i])

    return obs

def findV456(ctrl):
    k = 200
    max_v = 0.3
    orientation_quat_des = ctrl.desired_quat
    
    th90rad = -90*2*pi/360
    th = ctrl.robot_th.copy()
    th[0] = th[0] - th90rad
    
    orientation_quat = ctrl.tf_orientation.copy()
    if np.dot(orientation_quat_des, orientation_quat) < 0.0:
        orientation_quat = - orientation_quat
    err_quat = R.from_matrix( R.from_quat(orientation_quat).inv().as_matrix() * \
                              R.from_quat(orientation_quat_des).as_matrix() ).as_quat()
    err = np.dot(-R.from_quat(orientation_quat).as_matrix(), err_quat[0:3])

    sign = lambda x: copysign(1, x)
    v_alpha = - k * err[0]
    v_beta = - k * err[1]
    v_gamma = - k * err[2]
    
    v_rot = [v_alpha, v_beta, v_gamma]
    J_inv = np.linalg.inv(compute_ur_jacobian(th))
    J_inv = J_inv[3:6,3:6]
    v4, v5, v6 = np.matmul(J_inv, v_rot)
    
    v4 = v4 if abs(v4)<max_v else max_v*sign(v4)
    v5 = v5 if abs(v5)<max_v else max_v*sign(v5)
    v6 = v6 if abs(v6)<max_v else max_v*sign(v6)
    
    return v4, v5, v6

#Filter
action_cntr = 0
speedVect_size = 50
n_filteredAction = 6
speed_vect = [ [ 0.0 for _ in range(speedVect_size) ] 
                           for c in range(n_filteredAction) ]

def speed_filter(action):
    global action_cntr
    global speed_vect
    index = action_cntr % speedVect_size
    for i in range(n_filteredAction):
        speed_vect[i][index] = action[i]
    action_cntr += 1
    return [ np.mean(speed_vect[i]) for i in range(n_filteredAction) ]
    
def reset_speed_filter():
    global action_cntr
    global speed_vect
    action_cntr = 0
    speed_vect = [ [ 0.0 for _ in range(speedVect_size) ] 
                           for c in range(n_filteredAction) ]

def reset_robot_conf(ctrl):
    #TODO: controllare dir
    th90rad = -90*2*pi/360
    start_robot_conf = [0.0, -1.57, 1.57, -1.57, 4.71, 2.36]
    k = 1
    max_v = 0.05
    theta = np.array(ctrl.robot_th).copy()
    theta_target = start_robot_conf[0:3]
    theta[0] = theta[0] - th90rad
    #print(theta)
    #d1r = [  ]
    v = np.zeros(6)
    sign = lambda x: copysign(1, x)
    for i in range(3):
        #d1r = 0 if theta[i]<theta_target[i] else 1
        v[i] = - k * (theta[i]-theta_target[i]) #* (-1)**d1r
        v[i] = v[i] if abs(v[i])<max_v else max_v*sign(v[i])
        
    v[0] = 0
    v[3], v[4], v[5] = findV456(ctrl)
    return v

if __name__ == '__main__':
    try:
        
        #######################
        #init ROS stuff
        #######################
        
        #Nome nodo
        nodeName = 'robotControl'

        controller = Controller(nodeName=nodeName)
        rate = controller.rate
        
        reset_pos_robot = controller.start_pos_robot.copy()
        start_vel_robot = np.zeros(6)
        
        #######################
        #init DDPG stuff
        #######################
        
        load_checkpoint = True
        evaluate = True
        cancelMemory = False
        
        reductionFactor = 0.5
        other_red_factor = 0.05
        
        useFilter = True
        
        observation_shape = (33,)

        
        agent = Agent(input_dims=observation_shape, n_actions=3,
                      chkpt_dir='goodW/avoider',
                      memory_dir='tmp',
                      fc1=900, fc2=600, fc3=300,
                      reduction_factor=reductionFactor)        
        
        n_games = 30_000
        n_games += 1

        loadMemory = evaluate or cancelMemory

        if load_checkpoint:
            print('Loading model ...')
            n_steps = 0
            while n_steps <= agent.batch_size:
                observation = np.zeros(33)
                observation_ = np.zeros(33)
                reward = 0.0
                done = False
                agent.remember(observation, np.zeros(3), reward, observation_, done)
                n_steps += 1
            agent.learn()
            agent.my_load_models(evaluate=loadMemory)
            print('Loading completed:)')
        
        for i in range(n_games):
            if(rospy.is_shutdown()):
                break
            
            #Reset Routine
            controller.robot_vel_publish(start_vel_robot)
            rate.sleep()
            
            target_pos = controller.robot_th.copy()[0:3]
                     
            observation = observe(controller, np.zeros(6), target_pos)
            
            done = False

            count = 0
            
            while not done:
                if(rospy.is_shutdown()):
                    print('ROS is shutdown:(')
                    break;
                
                count += 1                
                
                action = agent.choose_action(observation, evaluate)
                v = np.zeros(6)
                for ii in range(3):
                    v[ii] = other_red_factor*action[ii]
                v[3], v[4], v[5] = findV456(controller)
                
                v = reset_robot_conf(controller)
                
                if useFilter:
                    v = speed_filter(v)
                
                #v = np.zeros(6)
                controller.robot_vel_publish(v)
                rate.sleep()
                
                observation = observe(controller, v, target_pos)
        
                print(v)        
        
    except rospy.ROSInterruptException:
        pass
    
    


