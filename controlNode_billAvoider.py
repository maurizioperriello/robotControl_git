#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Feb 23 16:18:27 2022

@author: mauri
"""

import rospy
from listener_classes import Controller
import numpy as np
from random import randint, uniform, choice
from ddpg_classes import Agent
from utils import pnt2line

def debugInfo(n):
    print(f'Ciao:) - {n}')
    
def wait(rate, n_ds):
    c = 0
    while(c<n_ds):
        rate.sleep()
        c+=1
    
def readPositionFile(file):
    position = np.genfromtxt(file)
    return position    
    
def observe_OLD(ctrl):
    theta_joints = np.array(ctrl.robot_pos).copy()
    EE_pos, EE_vel = ctrl.EE_pos.copy(), ctrl.EE_vel.copy()
    target_pos = np.array(ctrl.target_pos).copy()
    billLimbs = np.array(ctrl.billLimb_spatialPos).copy()
    limbSelector = [4, 6, 7] #testa, mano destra, mano sinistra
    obs = []
    
    for i in limbSelector:
        for j in range(3):
            obs.append(billLimbs[i][j])
    
    for i in range(3):
        obs.append(target_pos[i])
    
    """
    for i in range(3):
        obs.append(theta_joints[i])
    """
    for i in range(3):
        obs.append(EE_pos[i])
    
    for i in range(3):
        obs.append(EE_vel[i])
            
    return obs

def observe(ctrl, target_pos):
    
    #obs_dim = 32
    #In questo caso si considerano come punti del robot l'EE, i giunti del
    #polso e quello del gomito: questi 5 punti descrivono 4 segmenti, rispetto
    #ai quali si calcolano le distanze rispetto agli arti di Bill (testa e mani).
    #Nel vettore delle osservazioni sono inserite le 4 distanze minime (3 valori per
    #ogni vettore) tra ciascuno dei 4 segmenti e il più vicino arto di Bill.
    #[VEDI paper nella cartella "robot_paper"]
    
    theta_joints = np.array(ctrl.robot_pos).copy()
    EE_pos = ctrl.EE_pos.copy()
    EE_vel = ctrl.EE_vel.copy()
    object_pos = np.array(ctrl.target_pos).copy()
    #spatialPos_joints_robot = ctrl.joints_spatialPos.copy()
    billLimbs = np.array(ctrl.billLimb_spatialPos).copy()
    billLimbs_vel = np.array(ctrl.billLimb_spatialVel).copy()
    limbSelector = [4, 6, 7] #testa, mano destra, mano sinistra
    
    obs = []
    
    """
    points = []
    points.append(EE_pos)
    for i in range(5, 1, -1):
        points.append(spatialPos_joints_robot[i])
    
    for k in range(len(points)-1):
        d_limbs =  []
        imp_points = []
        for l in limbSelector:
            d, nearest_point = pnt2line(billLimbs[l], points[k], points[k+1])
            d_limbs.append(d)
            imp_points.append(nearest_point)
        index = d_limbs.index(min(d_limbs))
        for i in range(3):
            obs.append(billLimbs[index][i]-imp_points[index][i]) 
    """
    
    for l in limbSelector:
        for k in range(3):
            obs.append(billLimbs[l][k])
        for h in range(3):
            obs.append(billLimbs_vel[l][h])
            
    for i in range(3):
        obs.append(object_pos[i]) 
       
    for i in range(3):
        obs.append(object_pos[i]-EE_pos[i])     
       
    for i in range(3):
        obs.append(EE_pos[i])
            
    for i in range(3):
        obs.append(EE_vel[i])
    
    for i in range(3):
        obs.append(theta_joints[i])
    
    #print(obs)

    return obs

def give_reward(ctrl, v, old_v, obs, old_obs):
    dv_max = 0.26
    sft_d = 0.45 #distanza di sicurezza
    a = 10
    b = 100
    selfCollisionRobot = ctrl.selfCollision_bool
    globalCollisionRobot = ctrl.globalCollision_bool
    targetEE_vect = obs[21:24] # qui ci va il vettore (target_pos - EE_pos)
    target_d = np.linalg.norm(targetEE_vect)
    e = 1000 #if target_d>0.15 else 0
    #print(target_pos)
    """
    q = obs[21:24]
    q_old = old_obs[21:24]
    dq_mod = np.linalg.norm(np.subtract(q, q_old))
    """
    dv = [ abs(v[i]-old_v[i]) for i in range(3) ]
    c = [ 2 if dv[i]>dv_max else 0 for i in range(3) ]
    r = sum( [ x*y for (x, y) in zip(c, dv) ] )
    
    mod_v = 0
    for i in range(len(v)):
        mod_v = mod_v + v[i]**2
    mod_v = np.sqrt(mod_v)
    
    collision, max_distance_bool, d_min, d_vect = check_operator_collision(ctrl)

    d_r = np.array([ 1-d_vect[i]/sft_d if d_vect[i]<sft_d else 0
                    for i in range(len(d_vect))]).sum()

    reward = - e*target_d - a*mod_v - b*d_r #- r #a*dq_mod
    """
    print(f'target_d = {target_d}',
          f'mod_v = {mod_v}',
          f'd_r = {d_r}',
          f'r = {r}', sep='\n')
    """
    #se c'è una collisione
    if(collision):
        reward -= 500

    #se c'è una collisione individuata da CoppeliaSim
    if(selfCollisionRobot or globalCollisionRobot):
        reward -= 500
        
    #se l'operatore è molto lontano e la velocità impostata è alta
    if(max_distance_bool and mod_v > 0.3):
        reward -= 20
    
    return reward

def check_operator_collision(ctrl):
    #RIDURRE IL NUMERO DI PARAGONI (USARE index=[2,4]?)
    spatialPos_joints_robot = ctrl.joints_spatialPos.copy()
    point_op = np.array(ctrl.billLimb_spatialPos).copy()
    EE_pos = ctrl.EE_pos.copy()

    coll_d = 0.2 #minima distanza (di sicurezza) entro la quale si considera una collisione
    limbSelector = [4, 6, 7]
    points = []
    points.append(EE_pos)
    for i in range(5, 1, -1):
        points.append(spatialPos_joints_robot[i])

    d_min = 100 #distanza minima tra le varie distanze calcolate
    """
    l = len(point_op)
    check_d = [ False for _ in range(6*l-1) ]
    d_max = 0
    for k in range(l):
        ind = 6*k
        for i in range(5):
            d, _ = pnt2line(point_op[k], spatialPos_joints_robot[i], spatialPos_joints_robot[i+1])
            check_d[ind+i] = d <= coll_d
            if(d > d_max):
                d_max = d
            if((d < d_min) and (k in limbSelector)):
                d_min = d
    """
    d_min_vector = []
    check_d = []
    d_max = 0
    for k in range(len(points)-1):
        d_limbs =  []
        for l in range(len(point_op)):
            d, _ = pnt2line(point_op[l], points[k], points[k+1])
            check_d.append(d <= coll_d)
            if(d > d_max):
                d_max = d
            if(l in limbSelector):
                d_limbs.append(d)
                if d<d_min:
                    d_min = d
        d_min_vector.append(d_limbs[d_limbs.index(min(d_limbs))])
    distance_bool = d_min>0.6
    
    if(np.array(check_d).any()):
        print(f'd_vector_collision: {d_min_vector}')
    
    return np.array(check_d).any(), distance_bool, d_min, d_min_vector

def check_target_OLD_OLD(ctrl):
    safety_d = 0.65 #nello script di python con entrambi gli agenti si porrà più bassa
    done = False    
    _, _, d_min, _ = check_operator_collision(ctrl)
    if(d_min >= safety_d):
        done = True
    return done

def check_target_OLD(ctrl):
    done = False    
    collision, _, _, _ = check_operator_collision(ctrl)
    selfCollisionRobot = ctrl.selfCollision_bool
    globalCollisionRobot = ctrl.globalCollision_bool
    if collision or selfCollisionRobot or globalCollisionRobot:
        done = True
    return done

def check_target(ctrl):
    done = False
    EEpos = ctrl.EE_pos.copy()
    target_pos = np.array(ctrl.target_pos).copy()
    _, _, dmin, _ = check_operator_collision(ctrl)
    
    check_targetD = np.array([ target_pos[i] <= EEpos[i]+0.15 and target_pos[i] >= EEpos[i]-0.15 
                      for i in range(3) ]).all()
    check_opD = dmin >= 0.4
    
    if(check_targetD and check_opD):
        done = True  
    
    return done


def findV4(ctrl):
    #TODO: utilizzare l'orientamento dell'EE fornito dall'apposita funzione su CoppeliaSim
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
    #TODO: utilizzare l'orientamento dell'EE fornito dall'apposita funzione su CoppeliaSim
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
                                globalCollisionRobot_subs,
                                spatialPosFinalLinks_subs, table_pubTopic)
        rate = controller.rate
        
        reset_pos_robot = controller.start_pos_robot.copy()
        start_vel_robot = np.zeros(6)
      
        #Bill
        #Limiti per le posizioni delle mani
        xLim_right = [0.25, 0.7] #[0.25, 0.8]
        yLim_right = [-0.7, 0.0]
        xLim_left = [0.25, 0.7] #[0.25, 0.8]
        yLim_left = [0.0, 0.7]
        zLim = 1.0
        
        #Table
        resetTablePos = [3.0,3.0,3.0,3.0]
        startTablePos = [0.0,0.0,0.0,0.0]
        
        
        #######################
        #init DDPG stuff
        #######################
        
        load_checkpoint = True
        evaluate = False
        
        #noise start at 0.1
        noise = 0.05
        
        observation_shape = (33,)
        #  [head_x, head_y, head_z,
        #   rxHand_x, rxHand_y, rxHand_z,
        #   lxHand_x, lxHand_y, lxHand_z,
        #   target_x, target_y, target_z,
        #   th_joints1, th_joints2, th_joints3, <-NON PiÙ
        #   EE_x, EE_y, EE_z, 
        #   EE_vx, EE_vy, EE_vz]
        
        agent = Agent(input_dims=observation_shape, n_actions=3,
                      chkpt_dir='tmp/ddpg_billAvoider',
                      memory_dir='tmp/memory_billAvoider', noise=noise,
                      fc1=800, fc2=600, fc3=300)        
        
        best_score = 0
        n_games = 10_000
        n_games += 1
        limit_count = 1500
        score_history = []
        
        position_file = 'tmp/configuration_simplerSearcher.csv'
        startPosition = readPositionFile(position_file)

        if load_checkpoint:
            print('Loading model ...')
            n_steps = 0
            while n_steps <= agent.batch_size:
                observation = observe(controller, np.zeros(3))
                #action = [ randint(1, 10)/10 for _ in range(6) ]
                #controller.robot_vel_publish(action)
                #rate.sleep()
                observation_ = observe(controller, np.zeros(3))
                reward = give_reward(controller, np.zeros(3), np.zeros(3), observation, observation_)
                done = False
                agent.remember(observation, np.zeros(3), reward, observation_, done)
                n_steps += 1
            agent.learn()
            agent.my_load_models(evaluate=evaluate)
            print('Loading completed:)')
        
        for i in range(n_games):
            if(rospy.is_shutdown()):
                break
            
            #Reset Routine
            controller.robot_vel_publish(start_vel_robot)
            #rate.sleep()
            
            controller.billHandPos_publishFun([2, 0, 0, 0]) #reset position
            #rate.sleep()
            wait(rate, 300)
            
            controller.table_publish(resetTablePos)
            #rate.sleep()
            controller.robot_pos_publish()
            #rate.sleep()
            resetCompleted = False
            rememberIteration = True
            c = 0
            while(not resetCompleted):
                r_pos = np.array(controller.robot_pos).copy()
                resetCompleted = np.array(
                    [ r_pos[i]>=reset_pos_robot[i]-0.1 and r_pos[i]<=reset_pos_robot[i]+0.1 for i in range(6) ]
                    ).all()
                c += 1
                if(c == 500_000):
                    break;
            if not resetCompleted:
                rememberIteration = False
            controller.table_publish(startTablePos)
            #rate.sleep()
                
            startConfig = startPosition[choice(range(len(startPosition)))]
            controller.robot_pos_publish(startConfig, False)
            #rate.sleep()
            wait(rate, 100)
            target_pos = controller.EE_pos.copy()
            """
            target_x = round(uniform(0.5, 1.0), 2)
            target_y = round(uniform(-0.75, 0.75), 2)
            target_z = 0.41
            object_position = [target_x, target_y, target_z]
            """
            controller.target_pos_publish(target_pos)
            #rate.sleep()
            
            observation = observe(controller, target_pos)
            
            done = False
            score = 0.0
            
            count = 0
            
            old_action = np.zeros(6)
            
                        
            while not done:
                if(rospy.is_shutdown()):
                    print('ROS is shutdown:(')
                    break;
                
                count += 1                
                
                action = agent.choose_action(observation, evaluate)
                v = np.zeros(6)
                for ii in range(3):
                    v[ii] = action[ii]
                v[3] = findV4(controller)                
                v[4] = findV5(controller)
                controller.robot_vel_publish(v)
                #rate.sleep()
                
                if(count % 300 == 0 or count == 1):
                    #controller.billHandPos_publishFun([2, 0, 0, 0])
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
                    
                if(count % 500 == 0):
                    controller.billHandPos_publishFun([2, 0, 0, 0])
                    #rate.sleep()
                    
                observation_ = observe(controller, target_pos)
                reward = give_reward(controller, action, old_action, observation_, observation)
                old_action = action

                #done = check_target(controller)
                if done:
                    reward += 1000
                
                score += reward
                
                agent.remember(observation, action, reward, observation_, done)
                
                if not evaluate and count % 50 == 0:
                    agent.learn()
                
                observation = observation_
                
                print(count)
                #print(action)
                if(count == limit_count):
                    break;
                
            score_history.append(score)
            avg_score = np.mean(score_history[-100:])
            if avg_score > best_score:
                best_score = avg_score
            if not evaluate and not rospy.is_shutdown():
                controller.robot_vel_publish(start_vel_robot)
                agent.save_models(i)
            print('Episode ', i, 'score %.1f' % score, 'avg score %.1f' % avg_score,
              '---------------')                
                
        
    except rospy.ROSInterruptException:
        pass
    
    


