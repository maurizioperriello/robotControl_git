#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Mar  3 15:22:58 2022

@author: mauri
"""

############################################################################
#
# COMPLETE CONTROLLER
#
# Questo script serve per unificare i due agenti, il searcher e l'avoider,
# e testarli in un ambiente con Bill che si muove casualmente e il target
# da raggiungere
#
############################################################################

import rospy
from listener_classes import Controller
from random import randint, uniform
import numpy as np
from ddpg_classes import Agent
from random import randint, uniform, choice
from utils import pnt2line

def wait(rate, n_ds):
    c = 0
    while(c<n_ds):
        rate.sleep()
        c+=1

def observe_searcher(ctrl):
    EE_pos, EEvel = ctrl.EE_pos.copy(), ctrl.EE_vel.copy()
    #theta_joints = np.array(ctrl.robot_pos).copy()
    target_pos = np.array(ctrl.target_pos).copy()
    obs = []

    for j in range(3):
        obs.append(target_pos[j])
        
    for k in range(3):
        obs.append(EE_pos[k])
      
    if(np.size(EEvel) == 3):
        for i in range(3):
            obs.append(EEvel[i])
    else:
        for i in range(3):
            obs.append(0.0)
            
    return obs

def observe_avoider(ctrl):
    #theta_joints = np.array(ctrl.robot_pos).copy()
    EE_pos, EE_vel = ctrl.EE_pos.copy(), ctrl.EE_vel.copy()
    billLimbs = np.array(ctrl.billLimb_spatialPos).copy()
    limbSelector = [4, 6, 7] #testa, mano destra, mano sinistra (in realtà un punto tra la mano e il gomito)
    obs = []
    
    for i in limbSelector:
        for j in range(3):
            obs.append(billLimbs[i][j])
    for i in range(3):
        obs.append(EE_pos[i])
    #for i in range(3):
        #obs.append(theta_joints[i])
    for i in range(3):
        obs.append(EE_vel[i])
            
    return obs

def check_operator_collision(ctrl):
    #RIDURRE IL NUMERO DI PARAGONI (USARE index=[2,4]?)
    spatialPos_joints_robot = ctrl.joints_spatialPos.copy()
    point_op = np.array(ctrl.billLimb_spatialPos).copy()
    sft_d = 0.3 #minima distanza (di sicurezza) entro la quale si considera una collisione
    limbSelector = [4, 6, 7] 
    
    d_min = 100 #distanza minima tra le varie distanze calcolate
    
    l = len(point_op)
    check_d = [ False for _ in range(6*l-1) ]
    d_max = 0
    for k in range(l):
        ind = 6*k
        for i in range(5):
            d, _ = pnt2line(point_op[k], spatialPos_joints_robot[i], spatialPos_joints_robot[i+1])
            check_d[ind+i] = d <= sft_d
            if(d > d_max):
                d_max = d
            if((d < d_min) and (k in limbSelector)):
                d_min = d
    
    distance_bool = d_max>0.6
    
    return np.array(check_d).any(), distance_bool, d_min

def check_target(ctrl):
    delta = 0.02
    max_modV = 0.3 # = 30 cm/s    
    
    EEpos, EEvel = ctrl.EE_pos.copy(), ctrl.EE_vel.copy() #se possibile, si effettua sempre la copia dei valori della classe controller
    objPos = np.array(ctrl.target_pos).copy()
    finalLinks = ctrl.finalLinks_spatialPos.copy()
    joints_spatialPos = ctrl.joints_spatialPos
      
    modV = 0
    for v in range(3):
        modV = modV + EEvel[v]**2
    modV = np.sqrt(modV)
    
    EE_angles = [finalLinks[0][0] - finalLinks[1][0], finalLinks[0][1] - finalLinks[1][1]]
    z_check = finalLinks[1][2] < finalLinks[0][2]
    check_bools_pos = [ objPos[i] <= EEpos[i]+0.15 and objPos[i] >= EEpos[i]-0.15 
                   for i in range(3) ]
    check_bools_vel = modV <= max_modV
    check_angles = [ EE_angles[i]>=-delta and EE_angles[i]<=delta
                    for i in range(2) ]
    check_EE_pos = joints_spatialPos[4][0] > joints_spatialPos[3][0] #in questo modo l'EE non è girato ma rivolto in avanti
    
    check_bools = np.append(check_bools_pos, check_bools_vel)
    check_bools = np.append(check_bools, check_angles)
    check_bools = np.append(check_bools, z_check)
    check_bools = np.append(check_bools, check_EE_pos)

    check = True
    for i in range(np.size(check_bools)):
        check = check and check_bools[i]
    return check

def findV4(ctrl):
    #utilizzare l'orientamento dell'EE fornito dall'apposita funzione su CoppeliaSim
    k = 30 #si parte da 10, funziona ma è lento
    finalLinks = ctrl.finalLinks_spatialPos.copy()
    EE_alpha = finalLinks[0][0] - finalLinks[1][0] #differenza tra le coordinate x del terzultimo link e dell'EE
    EE_beta = finalLinks[0][1] - finalLinks[1][1]
    z_check = finalLinks[1][2] < finalLinks[0][2]
    d = np.sqrt(EE_alpha**2 + EE_beta**2)
    d1r = 1 if finalLinks[0][0]>finalLinks[1][0] else 0 #perché "dir" è meglio non usarlo
    v4 = k * d * (-1)**d1r if z_check else 0.7
    return v4

def findV5(ctrl):
    #utilizzare l'orientamento dell'EE fornito dall'apposita funzione su CoppeliaSim
    k = 30
    finalLinks = ctrl.finalLinks_spatialPos.copy()
    EE_alpha = finalLinks[0][0] - finalLinks[1][0] #differenza tra le coordinate x del terzultimo link e dell'EE
    EE_beta = finalLinks[0][1] - finalLinks[1][1]
    z_check = finalLinks[1][2] < finalLinks[0][2]
    d = np.sqrt(EE_alpha**2 + EE_beta**2)
    d1r = 0 if finalLinks[0][1]>finalLinks[1][1] else 1 #perché "dir" è meglio non usarlo
    v5 = k * d * (-1)**d1r if z_check else 0.7
    return v5

if __name__ == '__main__':
    try:
        
        #######################
        #init ROS stuff
        #######################
        
        #Nome nodo
        nodeName = 'robotControl'
        
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
        
        controller = Controller(nodeName, robotPos_pubTop, robotVel_pubTop, targetPos_pubTop,
                                opPos_pubTop, opVel_pubTop, resetRobot_pubTop,
                                op_subs, EEpos_subs, EEvel_subs, targetPos_subs,
                                robot_subs, spatialPosRobotJoints_subs, selfCollisionRobot_subs,
                                globalCollisionRobot_subs, spatialPosFinalLinks_subs,
                                table_pubTopic)
        rate = controller.rate #non viene utilizzata
        
        reset_pos_robot = controller.start_pos_robot.copy() #posizione iniziale del robot
        start_vel_robot = np.zeros(6) #velocità iniziale del robot
        
        resetTablePos = [3.0,3.0,3.0,3.0]
        startTablePos = [0.0,0.0,0.0,0.0]
        
        #Bill
        #Limiti per le posizioni delle mani
        xLim_right = [0.25, 0.8]
        yLim_right = [-0.7, 0.0]
        xLim_left = [0.25, 0.8]
        yLim_left = [0.0, 0.7]
        zLim = 1.0
        
        #######################
        #init DDPG stuff
        #######################
        
        obs_shape_searcher = (9,)  # [target_x-EEx, target_y-EEy, target_z-EEz,
                                   #  EE_vx, EE_vy, EE_vz]
        obs_shape_avoider = (15,)
        #  [head_x, head_y, head_z,
        #   rxHand_x, rxHand_y, rxHand_z,
        #   lxHand_x, lxHand_y, lxHand_z,
        #   EE_x, EE_y, EE_z,
        #   EE_vx, EE_vy, EE_vz]
        
        searcher = Agent(input_dims=obs_shape_searcher, n_actions=3,
                      chkpt_dir='tmp/ddpg_simplerSearcher',
                      memory_dir='tmp/memory_simplerSearcher')
        avoider = Agent(input_dims=obs_shape_avoider, n_actions=6,
                      chkpt_dir='tmp/ddpg_billAvoidance',
                      memory_dir='tmp/memory_billAvoider')
        
        n_games = 8000
        n_games += 1 #per far si che al penultimo game si salvi la memoria (viene salvata ogn 100 episode, per non rallentare troppo il processo)
        limit_count = 4000 #numero di iterazioni massime per episode
        
        sft_d = 0.35 # distanza di sicurezza dall'operatore, se si è più vicini si attiva la procedura di allontanamento
        
        n_steps = 0
        while n_steps <= searcher.batch_size:
            observation_s = observe_searcher(controller)
            observation_a = observe_avoider(controller)
            #action = [ randint(1, 10)/10 for _ in range(3) ]
            #controller.robot_vel_publish(action)
            #rate.sleep()
            observation_s_ = observe_searcher(controller)
            observation_a_ = observe_avoider(controller)
            reward = -1.0
            done = False
            searcher.remember(observation_s, np.zeros(3), reward, observation_s_, done)
            avoider.remember(observation_a, np.zeros(6), reward, observation_a_, done)
            n_steps += 1
        searcher.learn()
        avoider.learn()
        searcher.my_load_models()
        avoider.my_load_models()
        print('Loading completed:)')
        
        for ep in range(n_games):
            if(rospy.is_shutdown()):
                break
            
            #Reset Routine
            controller.billHandPos_publishFun([2, 0, 0, 0]) #reset position
            #rate.sleep()
            wait(rate, 30)
            
            controller.robot_vel_publish(start_vel_robot)
            #rate.sleep()
            controller.table_publish(resetTablePos)
            #rate.sleep()
            controller.robot_pos_publish()
            #rate.sleep()
            resetCompleted = False
            c = 0
            while(not resetCompleted):
                r_pos = np.array(controller.robot_pos).copy() #robot position
                resetCompleted = np.array(
                    [ r_pos[i]>=reset_pos_robot[i]-0.1 and r_pos[i]<=reset_pos_robot[i]+0.1 for i in range(6) ]
                    ).all()
                c += 1
                if(c == 500_000):
                    break;
            controller.table_publish(startTablePos)
            #rate.sleep()
                    
            target_x = round(uniform(0.5, 1.0), 2)
            target_y = round(uniform(-0.75, 0.75), 2)
            target_z = 0.41
            object_position = [target_x, target_y, target_z]
            
            controller.target_pos_publish(object_position)
            #rate.sleep()
            
            obs_sea = observe_searcher(controller)
            obs_av = observe_avoider(controller)
            
            done = False
            count = 0
            
            while not done:
                if(rospy.is_shutdown()):
                    print('ROS is shutdown:(')
                    break
                    
                count+=1
                
                if(count % 200 == 0 or count == 1):
                    hand = choice([0,1])
                    if hand==0: 
                        #right hand
                        hand_pos = [round(uniform(xLim_right[0], xLim_right[1]), 2),
                                    round(uniform(yLim_right[0], yLim_right[1]), 2),
                                    zLim]
                    else: 
                        #left hand
                        hand_pos = [round(uniform(xLim_left[0], xLim_left[1]), 2),
                                    round(uniform(yLim_left[0], yLim_left[1]), 2),
                                    zLim]
                    bill_cmd = np.append(hand, hand_pos)
                    controller.billHandPos_publishFun(bill_cmd)
                    #rate.sleep()
                
                v = np.zeros(6)
                v_sea = np.zeros(6)
                action_sea = searcher.choose_action(obs_sea, evaluate=True)
                for ii in range(3):
                    v_sea[ii] = action_sea[ii]
                v_sea[3] = findV4(controller)                
                v_sea[4] = findV5(controller)
                v_av = avoider.choose_action(obs_av, evaluate=True)
                _, _, dmin = check_operator_collision(controller)
                a = 1
                b = 0
                if dmin<sft_d:
                    a = dmin
                    b = 1-dmin
                for k in range(6):
                    v[k] = (a*v_sea[k] + b*v_av[k])/(a + b)
                v[5] = 0 #gdl ininfluente
                
                controller.robot_vel_publish(v) #invio dell'azione a CoppeliaSim
                #rate.sleep()
                
                obs_sea = observe_searcher(controller)
                obs_av = observe_avoider(controller)
                done = check_target(controller)
                
                print(count)
                if(count == limit_count):
                    break;
        
            print(f'End of Episode {ep}',
                  '----------------------',
                  sep='\n')
            
    except rospy.ROSInterruptException:
        pass