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

EE_oldPos = np.zeros(3)
oldTime = 0.0

def debugInfo(n):
    print(f'Ciao:) - {n}')

def observe(ctrl):
    EE_pos, EEvel = ctrl.EE_pos.copy(), ctrl.EE_vel.copy()
    target_pos = np.array(ctrl.target_pos).copy()
    obs = []
    time = rospy.get_time()
    
    global EE_oldPos
    global oldTime

    delta_t = time - oldTime
    oldTime = time
    EEvel = [ (EE_pos[i]-EE_oldPos[i])/delta_t for i in range(3) ]
    EE_oldPos = EE_pos

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


def check_target(ctrl):
    delta = 0.1 
    d_goal = 0.15
    
    alpha, beta = ctrl.EE_orientation.copy()[0], ctrl.EE_orientation.copy()[1]
    alpha_target, beta_target = -1.57, 4.71
    
    EEpos = ctrl.EE_pos.copy() #se possibile, si effettua sempre la copia dei valori della classe controller
    objPos = np.array(ctrl.target_pos).copy()
    
    d = 0
    for i in range(np.size(objPos)):
        d = d + (objPos[i] - EEpos[i])**2
    d = np.sqrt(d)

    check_bools_pos = d <= d_goal and EEpos[2] >= objPos[2]
    check_angles = [ alpha-alpha_target<=delta and alpha-alpha_target>=-delta,
                     beta-beta_target<=delta and beta-beta_target>=-delta]

    check_bools = np.append(check_bools_pos, check_angles)

    return np.array(check_bools).all()

        
def findV45_OLD_OLD(ctrl):
    #TODO: controllare dir e k!!!
    k = 10
    max_v = 0.5
    #alpha, beta = ctrl.EE_orientation.copy()[0], ctrl.EE_orientation.copy()[1]
    alpha, beta = ctrl.robot_th[3], ctrl.robot_th[4]
    alpha_target, beta_target = -1.57, 4.71
    d1r4 = 0 if alpha < alpha_target else 1
    d1r5 = 1 if beta < beta_target else 0
    #mag = np.sqrt((alpha-alpha_target)**2 + (beta-beta_target)**2)
    mag4 = alpha-alpha_target
    mag5 = beta-beta_target
    v4 = max_v * tanh(mag4/k) * (-1)**d1r4 # if ... else 0.4
    v5 = max_v * tanh(mag5/k) * (-1)**d1r5 # if ... else 0.4
    return v4, v5

def findV45_OLD(ctrl):
    #TODO: controllare dir e k!!!
    k = 0.1
    max_v = 0.4
    #Ruotare sistema di riferimento di un angolo pari a quello della base, così
    #da valutare gli angoli dell'EE come se il manipolatore fosse sempre fermo centralmente
    alpha, beta = ctrl.EE_orientation.copy()[0], ctrl.EE_orientation.copy()[1]
    
    """
    th0 = ctrl.robot_th[0] - (-90*2*pi/360)
    EE_orientation = ctrl.EE_orientation.copy()
    R_matrix = np.array([[cos(th0), -sin(th0), 0],
                         [sin(th0),  cos(th0), 0],
                         [    0,            0, 1]])
    EE_orientation = np.matmul(R_matrix, EE_orientation)
    alpha, beta = EE_orientation[0], EE_orientation[1]
    """
    print(f'alpha = {alpha}',
          f'beta = {beta}', sep='\n')
    alpha_target, beta_target = 3.13, 0.0
    d1r5 = 0 if alpha < alpha_target else 1
    d1r4 = 0 if beta < beta_target else 1
    mag = np.sqrt((alpha-alpha_target)**2 + (beta-beta_target)**2)
    mag5 = alpha-alpha_target
    mag4 = beta-beta_target
    v4 = max_v * tanh(mag/k) * (-1)**d1r4 # if ... else 0.4
    v5 = max_v * tanh(mag5/k) * (-1)**d1r5 # if ... else 0.4
    return v4, v5


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

old_lastPoint = np.zeros(3)

def findV45(ctrl):
    global old_lastPoint
    amp = 1
    k = 10
    max_v = 0.3
    lastLinkPoint = ctrl.tf_translation
    if old_lastPoint[0] == 0.0 and old_lastPoint[1] == 0.0:
        old_lastPoint = np.array(lastLinkPoint).copy()
    if lastLinkPoint[0]>5*old_lastPoint[0] or lastLinkPoint[1]>5*old_lastPoint[1]:
        lastLinkPoint = np.array(old_lastPoint).copy()
    else:
        old_lastPoint = np.array(lastLinkPoint).copy()
    EE_pos = ctrl.EE_pos
    finalLinks = [EE_pos, lastLinkPoint]
    print(f'lastLi = {lastLinkPoint}',
          f'EE_pos = {EE_pos}', sep='\n')
    EE_alpha = finalLinks[0][0] - finalLinks[1][0]
    EE_beta = finalLinks[0][1] - finalLinks[1][1]
    z_check = finalLinks[1][2] < finalLinks[0][2]
    d = np.sqrt((amp*EE_alpha)**2 + (amp*EE_beta)**2)
    d1r4 = 1 if finalLinks[0][0]>finalLinks[1][0] else 0
    d1r5 = 0 if finalLinks[0][1]>finalLinks[1][1] else 1
    v4 = k * d * (-1)**d1r4 # if z_check else 0.3
    v5 = k * d * (-1)**d1r5 # if z_check else 0.3
    
    sign = lambda x: copysign(1, x)
    v4 = v4 if abs(v4)<max_v else max_v*sign(v4)
    v5 = v5 if abs(v5)<max_v else max_v*sign(v5)
    
    return v4, v5


#Filter
action_cntr = 0
speedVect_size = 5
speed_vect = [ [ 0.0 for _ in range(speedVect_size) ] 
                           for c in range(5) ]

def speed_filter(action):
    global action_cntr
    global speed_vect
    index = action_cntr % speedVect_size
    for i in range(3):
        speed_vect[i][index] = action[i]
    action_cntr += 1
    return [ np.mean(speed_vect[i]) for i in range(5) ]
    
def reset_speed_filter():
    global action_cntr
    global speed_vect
    action_cntr = 0
    speed_vect = [ [ 0.0 for _ in range(speedVect_size) ] 
                           for c in range(5) ]



if __name__ == '__main__':
    try:            
        
        #######################
        #init ROS stuff
        #######################
        
        controller = Controller()
        #controllerSim = ControllerSim()
        rate = controller.rate #non viene utilizzata
        
        start_vel_robot = np.zeros(6) #velocità iniziale del robot
        
        
        #######################
        #init DDPG stuff
        #######################
        
        load_checkpoint = True #se True, carica i pesi e la memoria salvati
        evaluate = True #se True, non effettua il learn né il salvataggio dei dati
        
        useFilter = False
        
        reduction_factor = 0.5 #fattore di riduzione delle velocità del robot
        other_red_factor = 0.5 #ulteriore fattore di riduzione         
                             
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
                observation = observe(controller)
                action = [0.3, 0.3, 0.3]
                observation_ = observe(controller)
                reward = 0
                done = check_target(controller)
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
            
            th = controller.robot_th
            th[0] = th[0] - (-90*2*pi/360)
            #controllerSim.robot_pos_publish(th)
            
            #controllerSim.target_pos_publish(controller.target_pos)
            
            observation = observe(controller)
            
            #ciclo effettivo dell'episode
            while not done:
                if(rospy.is_shutdown()):
                    print('ROS is shutdown:(')
                    break;   
                
                v = np.zeros(6)
                action = agent.choose_action(observation, evaluate) #calcolo dell'azione
                for ii in range(3):
                    v[ii] = other_red_factor*action[ii]
                #v[3], _ = findV45(controller)   
                #v[3], v[4] = other_red_factor*v[3], other_red_factor*v[4]
 
                if useFilter:
                    v[0:5] = speed_filter(v[0:5])
                #print(controller.EE_orientation)
                #v[0:3] = np.zeros(3)
                #v[3] = -0.5
                controller.robot_vel_publish(v) #invio dell'azione a CoppeliaSim
                #controllerSim.robot_vel_publish(v)
                rate.sleep()   
                
                done = check_target(controller)
                
                if done:
                    print('Done:)))')
                    controller.robot_vel_publish(np.zeros(6))
                    rate.sleep() 
                
                observation = observe(controller) #lo stato futuro diventa quello presente
                
                
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
                print(f'EE_vel = {observation[9:12]}')
        
        w = 0
        while w < 30:
            print('Stopping:)')
            controller.robot_vel_publish(start_vel_robot)
            rate.sleep()
            w +=1
        print('Finish:)')
        
    except rospy.ROSInterruptException:
        pass
    
    
    