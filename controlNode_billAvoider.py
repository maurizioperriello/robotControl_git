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
    
def observe(ctrl):
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

def give_reward(ctrl, v, oldv):
    dv_max = 0.7    
    a = 1 
    selfCollisionRobot = ctrl.selfCollision_bool
    globalCollisionRobot = ctrl.globalCollision_bool
    
    mod_v = 0
    for i in range(len(v)):
        mod_v = mod_v + v[i]**2
    mod_v = np.sqrt(mod_v)
    collision, max_distance_bool, d_min = check_operator_collision(ctrl)
    
    if(d_min != 0):
        reward = -a/d_min
    else:
        reward = -1_000_000 
    
    #se c'è una collisione
    if(collision):
        reward -= 300
    
    #se c'è una variazione troppo grande di velocità
    velCheck = [ abs(v[i]-oldv[i])>dv_max for i in range(6) ]
    if(np.array(velCheck).any()):
        reward -= 100
    
    #se c'è una collisione individuata da CoppeliaSim
    if(selfCollisionRobot or globalCollisionRobot):
        reward -= 300
    
    #se l'operatore è molto lontano e la velocità impostata è alta
    if(max_distance_bool and mod_v > 0.3):
        reward -= 200
    
    return reward


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
    safety_d = 0.7 #nello script di python con entrambi gli agenti si porrà più bassa
    done = False    
    _, _, d_min = check_operator_collision(ctrl)
    if(d_min >= safety_d):
        done = True
    return done


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
        xLim_right = [0.25, 0.8]
        yLim_right = [-0.7, 0.0]
        xLim_left = [0.25, 0.8]
        yLim_left = [0.0, 0.7]
        zLim = 1.0
        
        #######################
        #init DDPG stuff
        #######################
        
        load_checkpoint = True
        evaluate = False
        
        #noise start at 0.1
        noise = 0.01
        
        observation_shape = (15,)
        #  [head_x, head_y, head_z,
        #   rxHand_x, rxHand_y, rxHand_z,
        #   lxHand_x, lxHand_y, lxHand_z,
        #   EE_x, EE_y, EE_z,
        #   EE_vx, EE_vy, EE_vz]
        
        agent = Agent(input_dims=observation_shape, n_actions=6,
                      chkpt_dir='tmp/ddpg_billAvoidance',
                      memory_dir='tmp/memory_billAvoider', noise=noise)        
        
        best_score = 0
        n_games = 10_000
        n_games += 1
        limit_count = 3000
        score_history = []
        
        position_file = 'tmp/configuration_simplerSearcher.csv'
        startPosition = readPositionFile(position_file)

        if load_checkpoint:
            print('Loading model ...')
            n_steps = 0
            while n_steps <= agent.batch_size:
                observation = observe(controller)
                action = [ randint(1, 10)/10 for _ in range(6) ]
                controller.robot_vel_publish(action)
                #rate.sleep()
                observation_ = observe(controller)
                reward = give_reward(controller, np.zeros(6), np.zeros(6))
                done = False
                agent.remember(observation, action, reward, observation_, done)
                n_steps += 1
            agent.learn()
            agent.my_load_models()
            print('Loading completed:)')
        
        for i in range(n_games):
            if(rospy.is_shutdown()):
                break
            
            #Reset Routine
            controller.robot_vel_publish(start_vel_robot)
            #rate.sleep()
            
            controller.billHandPos_publishFun([2, 0, 0, 0]) #reset position
            #rate.sleep()
            wait(rate, 30)
            
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
                
            startConfig = startPosition[choice(range(len(startPosition)))]
            controller.robot_pos_publish(startConfig, False)
            #rate.sleep()
            wait(rate, 10)
            
            observation = observe(controller)
            
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
                controller.robot_vel_publish(action)
                #rate.sleep()
                
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
                
                observation_ = observe(controller)
                reward = give_reward(controller, action, old_action)
                old_action = action

                done = check_target(controller)
                
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
    
    


