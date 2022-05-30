#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue May 17 18:20:38 2022

@author: mauri
"""

import rospy
from controller_real_class import Controller
from listener_classes import Controller as ControllerSim
import numpy as np
from numpy import tanh
from ddpg_classes import Agent
from math import pi, sin, cos, copysign
from utils import AH, compute_ur_jacobian
from scipy.spatial.transform import Rotation as R


EE_oldPos = np.zeros(3)
oldTime = 0.0

def debugInfo(n):
    print(f'Ciao:) - {n}')

def observe(ctrl, action):
    #EE_pos, EEvel2 = ctrl.EE_pos.copy(), ctrl.EE_vel.copy()
    EE_pos = ctrl.tf_translation.copy() #NON GLI PIACE PER NIENTE!!! #E INVECE È GIUSTO!!!
    target_pos = np.array(ctrl.target_pos).copy()
    obs = []
    
    
    #print(f'EE_pos = {EE_pos}')
    #Per passare dalla posizione del TCP a quella dell'EE,
    #come in simulazione
    T_d = np.matrix(np.identity(4), copy=False)
    T_d[2,3] = 0.195
    EE_pos.append(1.0)
    EE_pos = np.dot(T_d, EE_pos)
    EE_pos = [ EE_pos[0,i] for i in range(3) ]
    #print(f'TCP_pos = {EE_pos}')
    
    
    """
    time = rospy.get_time() 
    global EE_oldPos
    global oldTime
    delta_t = time - oldTime
    oldTime = time
    EEvel = [ (EE_pos[i]-EE_oldPos[i])/0.01 for i in range(3) ]
    EE_oldPos = np.array(EE_pos).copy()
    """
    
    R_mat_ur10e = np.array([[0, -1, 0],
                            [1, 0, 0],
                            [0, 0, 1]])
    
    th90rad = -90*2*pi/360
    th = ctrl.robot_th.copy()
    #th = np.array(ctrl.robot_pos).copy()
    th[0] = th[0] - th90rad
    J = compute_ur_jacobian(th)
    EEvel = np.dot(J, action)[0:3]
    EEvel = np.matmul(R_mat_ur10e, EEvel)
    """
    print(f'EE_J = {EEvel}',
          f'EE_sim = {EEvel2}', sep='\n')
          #f'EE_calc = {EEvel}', sep='\n')
    """
    
    for j in range(3):
        obs.append(target_pos[j])
        
    for j in range(3):
        obs.append(target_pos[j]-EE_pos[j])
        
    for k in range(3):
        obs.append(EE_pos[k])
      
    if(np.size(EEvel) == 3):
        for i in range(3):
            obs.append(EEvel[i])
    else:
        for i in range(3):
            obs.append(0.0)
            
    return obs


def check_target(ctrl, pick, target):
    delta = 5.0 
    d_goal = 0.15
    
    
    alpha, beta = ctrl.tf_orientation.copy()[0]*360/(2*pi), \
                    ctrl.tf_orientation.copy()[1]*360/(2*pi)
    alpha_target, beta_target = -180.0, 0.0
    
    
    #EE_pos = ctrl.EE_pos.copy() #se possibile, si effettua sempre la copia dei valori della classe controller
    EE_pos = ctrl.tf_translation.copy()
    
    #Per passare dalla posizione del TCP a quella dell'EE,
    #come in simulazione
    T_d = np.matrix(np.identity(4), copy=False)
    T_d[2,3] = 0.195
    EE_pos.append(1.0)
    EE_pos = np.dot(T_d, EE_pos)
    EE_pos = [ EE_pos[0,i] for i in range(3) ]
    
    #objPos = np.array(ctrl.target_pos).copy()
    if pick:
        objPos = target
    else:
        objPos = np.array(ctrl.target_pos).copy()
    
    d = 0
    for i in range(np.size(objPos)):
        d = d + (objPos[i] - EE_pos[i])**2
    d = np.sqrt(d)

    check_bools_pos = d <= d_goal and EE_pos[2] >= objPos[2]
    check_angles = [ abs(alpha-alpha_target)<=delta,
                     abs(beta-beta_target)<=delta]

    check_bools = np.append(check_bools_pos, check_angles)

    print(d)


    return np.array(check_bools_pos).all()

  
def findV456(ctrl):
    k = 600
    max_v = 0.2
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

    #v_alpha = v_alpha if abs(v_alpha)<max_v else max_v*sign(v_alpha)
    #v_beta = v_beta if abs(v_beta)<max_v else max_v*sign(v_beta)
    #v_gamma = v_gamma if abs(v_gamma)<max_v else max_v*sign(v_gamma)
    
    v_rot = [v_alpha, v_beta, v_gamma]
    J_inv = np.linalg.inv(compute_ur_jacobian(th))
    J_inv = J_inv[3:6,3:6]
    v4, v5, v6 = np.matmul(J_inv, v_rot)
    
    v4 = v4 if abs(v4)<max_v else max_v*sign(v4)
    v5 = v5 if abs(v5)<max_v else max_v*sign(v5)
    v6 = v6 if abs(v6)<max_v else max_v*sign(v6)
    
    return v4, v5, v6


def computeSpatialPosJoints(ctrl):
    #TODO: rimuovere i 90deg dal giunto 0 prima di effettuare il calcolo!
    R_mat_ur10e = np.array([[0, -1, 0],
                            [1, 0, 0],
                            [0, 0, 1]])
    
    theta = np.array(ctrl.robot_th).copy()
    theta[0] = theta[0] - (-90*2*pi/360)
    th = np.matrix([ [theta[i]] for i in range(6) ])
    t_vect_EE = [-0.6, 0.0, 0.0]
    A_1 = AH(1, th, 0)
    A_2 = AH(2, th, 0)
    A_3 = AH(3, th, 0)
    A_4 = AH(4, th, 0)
    A_5 = AH(5, th, 0)
    T = A_1*A_2*A_3*A_4*A_5 
    pos_tmp = np.matmul(R_mat_ur10e, [ T[i,3] for i in range(3) ])
    spatialPos_joints_robot = [ [ pos_tmp[j]+t_vect_EE[j] for j in range(3) ]
                                for i in range(5) ]
    return spatialPos_joints_robot




#Filter
action_cntr = 0
speedVect_size = 15
n_filteredAction = 5
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



if __name__ == '__main__':
    try:            
        
        #######################
        #init ROS stuff
        #######################
        
        controller = Controller()
        #controllerSim = ControllerSim()
        rate = controller.rate
        
        start_vel_robot = np.zeros(6) #velocità iniziale del robot
        
        
        #######################
        #init DDPG stuff
        #######################
        
        load_checkpoint = True #se True, carica i pesi e la memoria salvati
        evaluate = True #se True, non effettua il learn né il salvataggio dei dati
        
        useFilter = True
        
        useGripper = False
        
        reduction_factor = 0.5 #fattore di riduzione delle velocità del robot
        other_red_factor = 0.1 #ulteriore fattore di riduzione         
                             
        observation_shape = (12,)  # [target_x-EEx, target_y-EEy, target_z-EEz,
                                  #  EE_vx, EE_vy, EE_vz]  
              
        agent = Agent(input_dims=observation_shape, n_actions=3,
                      chkpt_dir='goodW/searcher',
                      memory_dir='tmp',
                      reduction_factor=reduction_factor)


        best_score = 0
        n_games = 1


        if load_checkpoint:
            n_steps = 0
            while n_steps <= agent.batch_size:
                observation = [ 0.1 for _ in range(12) ]
                action = [0.3, 0.3, 0.3]
                observation_ = [ 0.1 for _ in range(12) ]
                reward = 0
                done = False
                agent.remember(observation, action, reward, observation_, done)
                n_steps += 1
            agent.learn()
            agent.my_load_models(evaluate=evaluate)
            print('Loading completed:)')
        
        #routine di training/evaluation
        for ep in range(n_games):
            if(rospy.is_shutdown()):
                break
            
            #Reset Routine
            controller.robot_vel_publish(start_vel_robot)
            rate.sleep()
            
            #th = controller.robot_th
            #print(th)
            """
            th[0] = th[0] - (-90*2*pi/360)
            controllerSim.robot_pos_publish(th)
            resetCompleted = False
            c = 0
            while(not resetCompleted):
                r_pos = np.array(controllerSim.robot_pos).copy()
                resetCompleted = np.array(
                    [ r_pos[i]>=th[i]-0.1 and r_pos[i]<=th[i]+0.1 for i in range(6) ]
                    ).all()
                c += 1
                if(c == 500_000):
                    break;
            controllerSim.target_pos_publish(controller.target_pos)
            """
            
            #controllerSim.target_pos_publish(controller.target_pos)
            #observation = observe(controllerSim, np.zeros(6))
            
            place_target = [0.0, -0.25, 0.20]
            
            observation = observe(controller, np.zeros(6))
            
            done = False
            pick = False
            finish = False
            finish2 = False
            
            #ciclo effettivo dell'episode
            while not finish2:
                if(rospy.is_shutdown()):
                    print('ROS is shutdown:(')
                    break;
                
                v = np.zeros(6)
                action = agent.choose_action(observation, evaluate) #calcolo dell'azione
                for ii in range(3):
                    v[ii] = other_red_factor*action[ii]
                v[3], v[4], v[5] = findV456(controller)
                
                if useFilter:
                    v[0:5] = speed_filter(v[0:5])

                if done or finish:
                    v = np.zeros(6)
                if finish:
                    finish2 = True

                #v[0:3] = np.zeros(3)
                controller.robot_vel_publish(v)
                #controllerSim.robot_vel_publish(v)
                rate.sleep()   
                #print(v)
                
                if done and not finish:
                    if useGripper:
                        controller.commandGripper(pick, not pick)
                    else:
                        print('Done:)')
                        rospy.sleep(5)
                    if pick:
                        finish = True
                    pick = True
                    done = False
                    
                done = check_target(controller, pick, place_target)
                
                if done:
                    print('Done:)))')
                    controller.robot_vel_publish(np.zeros(6))
                    rate.sleep() 
                
                observation = observe(controller, v) #lo stato futuro diventa quello presente
                
                if pick:
                   observation[0:3] = place_target
                   observation[3:6] = [ place_target[i]-observation[6+i]
                                        for i in range(3)]
                
                """
                print(f'th = {controller.robot_th}',
                      f'vel = {controller.robot_vel}',
                      f'EE_vel = {controller.EE_vel}',
                      f'obs = {observation}', sep='\n')
                
                print(f'EE_pos = {controller.EE_pos}',
                      f'target = {controller.target_pos}', sep='\n')
                """
                """
                print(f'obs = {controller.EE_vel}',
                      #f'robot_vel = {controller.robot_vel}',
                      f'vel_sim = {controllerSim.EE_vel}', sep='\n')
                """
                #print(f'EE_vel = {observation[9:12]}')
                #print(f'obs = {observation}')
        
        w = 0
        while w < 30:
            print('Stopping:)')
            controller.robot_vel_publish(start_vel_robot)
            rate.sleep()
            w +=1
        print('Finish:)')
        
    except rospy.ROSInterruptException:
        pass
    
    
    