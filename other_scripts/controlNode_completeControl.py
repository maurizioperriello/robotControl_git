#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Dec  3 09:23:26 2021

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
    
def observe(ctrl):
    EEpos, EEvel = ctrl.EE_pos.copy(), ctrl.EE_vel.copy()
    target_pos = np.array(ctrl.target_pos).copy()
    
    joints_spatialPos = ctrl.joints_spatialPos
    opPos = ctrl.operator_pos
    opVel = ctrl.operator_vel
    jointSelection = [1,2,3]
    
    obs_op = []
    obs_target = []
    
    for i in range(3):
        obs_op.append(opPos[i])
    for i in range(3):
        obs_op.append(opVel[i])
    for i in jointSelection:
        for j in range(3):
            obs_op.append(joints_spatialPos[i][j])
        
    if(np.size(EEpos) == 3):
        for i in range(3):
            obs_target.append(EEpos[i])
    else:
        for i in range(3):
            obs_target.append(0.0)
            
    if(np.size(EEvel) == 3):
        for i in range(3):
            obs_target.append(EEvel[i])
    else:
        for i in range(3):
            obs_target.append(0.0)
    
    for j in range(3):
        obs_target.append(target_pos[j])
            
    return obs_op, obs_target

def give_reward_targetSearcher(d_history, ctrl):
    #reward: viene dato sulla base della distanza dell'EE dal target ed è
    #sempre negativo
    #Il coefficiente "a" viene abbassato nel caso in cui l'EE sia in vicinanza
    #del target con basse velocità: questo per aiutare il robot ad arrivare in 
    #prossimità di questo lentamente, per conseguire l'obiettivo di afferrare 
    #l'oggetto
    a = 100
    
    objPos = np.array(ctrl.target_pos).copy()
    EEpos, EEvel = ctrl.EE_pos.copy(), ctrl.EE_vel.copy()
    d = 0
    for i in range(np.size(objPos)):
        d = d + (objPos[i] - EEpos[i])**2
    d = np.sqrt(d)

    if(d<=0.4):
        if(EEvel[0]>=-0.1 and EEvel[1]>=-0.1 and EEvel[2]>=-0.1 and
           EEvel[0]<=0.1 and EEvel[1]<=0.1 and EEvel[2]<=0.1):
            a = 10

    reward = -a*d

    d_history.append(d)
    #d_history non viene utilizzato, ma può tornare utile per visualizzare
    #lo storico delle distanze volta per volta    
    
    return reward, d_history

def give_reward_opAvoidance(ctrl, v, oldv):
    dv_max = 0.7    
    reward = 0 
    selfCollisionRobot = ctrl.selfCollision_bool
    globalCollisionRobot = ctrl.globalCollision_bool
    
    mod_v = 0
    for i in range(len(v)):
        mod_v = mod_v + v**2
    mod_v = np.sqrt(mod_v)
    
    collision, max_distance_bool = check_operator_collision(ctrl)
    
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

def check_target(ctrl):
    #funzione per controllare se il risultato è stato raggiunto o meno:
    #per farlo, bisogna arrivare in prossimità del target (sfera bianca) con
    #una velocità inferiore o ugule a 0.1 m/s
    EEpos, EEvel = ctrl.EE_pos.copy(), ctrl.EE_vel.copy() #se possibile, si effettua sempre la copia dei valori della classe controller
    objPos = np.array(ctrl.target_pos).copy()
    check_bools_pos = [ objPos[i] <= EEpos[i]+0.15 and objPos[i] >= EEpos[i]-0.15 
                   for i in range(3) ]
    check_bools_vel = [ EEvel[i] <= 0.1 and EEvel[i] >= -0.1
                   for i in range(3) ]
    check_bools = np.append(check_bools_pos, check_bools_vel)
    check = True
    for i in range(np.size(check_bools)):
        check = check and check_bools[i]
    return check

def check_operator_collision(ctrl):
    #funzione per individuare se ci sono collisioni tra l'operatore (pallina verde) e il robot
    spatialPos_joints = ctrl.joints_spatialPos.copy()
    opPos = ctrl.operator_pos.copy()
    min_d = 0.3 #minima distanza entro la quale si considera una collisione
    
    check_d = [ False for _ in range(6) ]
    d0, _ = pnt2line(opPos, [0,0,0], spatialPos_joints[0])
    d_max = d0
    check_d[0] = d0 <= min_d
    for i in range(5):
        d, _ = pnt2line(opPos, spatialPos_joints[i], spatialPos_joints[i+1])
        check_d[i+1] = d <= min_d
        if(d > d_max):
            d_max = d
        
    print(check_d)
    
    distance_bool = d_max>0.6
    
    return np.array(check_d).any(), distance_bool


def check_operator_limit(ctrl, lim, rate):
    #funzione per controllare se l'operatore esce dai limiti spaziali imposti:
    #se si, si impostano velocità diverse con segno opportuno per farlo rientrare
    op_pos, op_vel = np.array(ctrl.operator_pos).copy(), \
                        np.array(ctrl.operator_vel).copy()
    change = False
    if(np.size(op_vel) != 0):
        vx = op_vel[0] if op_vel[0]!=None else 0
        vy = op_vel[1] if op_vel[1]!=None else 0
        vz = op_vel[2] if op_vel[2]!=None else 0
    else:
        vx = 0
        vy = 0
        vz = 0
    if(op_pos[0]<=lim[0][0]):
        vx = round(uniform(0.1, 0.3), 2)
        change = True
    elif(op_pos[0]>=lim[0][1]):
        vx = round(uniform(-0.3, -0.1), 2)
        change = True
    if(op_pos[1]<=lim[1][0]):
        vy = round(uniform(0.1, 0.3), 2)
        change = True
    elif(op_pos[1]>=lim[1][1]):
        vy = round(uniform(-0.3, -0.1), 2)
        change = True
    if(op_pos[2]<=lim[2][0]):
        vz = round(uniform(0.1, 0.3), 2)
    elif(op_pos[2]>=lim[2][1]):
        vz = round(uniform(-0.3, -0.1), 2)
        change = True
    
    if(change):
        vel = [vx, vy, vz]
        ctrl.operator_vel_publish(vel)
        #rate.sleep()
        
def valuate_action(action_op, action_target, reward_op, reward_target):
    #action_op = azione per schivare l'operatore
    #action_target = azione per raggiungere l'obiettivo
    #reward_op = reward ottenuto dall'agente per schivare l'operatore
    #reward_target = reward ottenuto dall'agente per reggiungere il target
    total_reward = abs(reward_op) + abs(reward_target)
    action = (action_op*abs(reward_op) + action_target*abs(reward_target))/total_reward
    return action


if __name__ == '__main__':
    try:
        #######################
        #init ROS stuff
        #######################
        
        controller = Controller()
        rate = controller.rate
        
        reset_pos_robot = controller.start_pos_robot.copy()
        start_vel_robot = np.zeros(6)
        
        operator_limit = [[-0.7, 0.7], [-0.7, 0.7], [0.1, 0.7]]
        
        
        #######################
        #init DDPG stuff
        #######################
        
        observation_shape_op = (15,)        
        avoider = Agent(input_dims=observation_shape_op, n_actions=6,
                      chkpt_dir='tmp/ddpg_opAvoidance')
        
        observation_shape_target = (9,)
        searcher = Agent(input_dims=observation_shape_target, n_actions=6,
                      chkpt_dir='tmp/ddpg_targetHunter')
        
        best_score = 0
        n_games = 8000
        limit_count = 3000
        score_history = []
        load_checkpoint = True

        evaluate = False
        
        if load_checkpoint:
            print('Loading models...')
            n_steps = 0
            while n_steps <= avoider.batch_size:
                obs_op, obs_tar = observe(controller)
                action = [ randint(1, 10)/10 for _ in range(6) ]
                controller.robot_vel_publish(action)
                #rate.sleep()
                obs_op_, obs_tar_ = observe(controller)
                reward_avoider = give_reward_opAvoidance(controller, np.zeros(6), np.zeros(6))
                reward_target, _ = give_reward_targetSearcher([], controller)
                done = False
                avoider.remember(obs_op, action, reward_avoider, obs_op_, done)
                searcher.remember(obs_tar, action, reward_target, obs_tar_, done)
                n_steps += 1
            avoider.learn()
            avoider.my_load_models()
            searcher.learn()
            searcher.my_load_models()
            n_steps = 0            
            print('Loading completed:)')
        
        for i in range(n_games):
            if(rospy.is_shutdown()):
                break
            
            #Reset Routine
            controller.robot_vel_publish(start_vel_robot)
            #rate.sleep()
            
            controller.robot_pos_publish()
            #rate.sleep()
            resetCompleted = False
            c = 0
            while(not resetCompleted):
                r_pos = np.array(controller.robot_pos).copy()
                resetCompleted = np.array(
                    [ r_pos[i]>=reset_pos_robot[i]-0.1 and r_pos[i]<=reset_pos_robot[i]+0.1 for i in range(6) ]
                    ).all()
                c += 1
                if(c == 500_000):
                    break;
                
            
            start_pos_operator = [ round(uniform(0.5, 0.8), 2) for _ in range(3) ]
        
            start_vel_operator = [ round(uniform(-0.3, 0.3), 1) for _ in range(3) ] 
            
            controller.operator_pos_publish(start_pos_operator)
            #rate.sleep()
            controller.operator_vel_publish(start_vel_operator)
            #rate.sleep()
            
            
            object_position = [ round(uniform(0.5, 1.0), 2) for _ in range(2)]
            object_position[1] = object_position[1] * choice([-1,1])
            object_position.append(0.1)
            controller.target_pos_publish(object_position)
            #rate.sleep()
            
            observation_op, observation_target = observe(controller)
            
            done = False
            score = 0.0
            
            count = 0
            
            old_action = np.zeros(6)
            
            reward_op = -10
            reward_target = 0
                        
            while not done:
                if(rospy.is_shutdown()):
                    print('ROS is shutdown:(')
                    break;
                
                count += 1                
                
                action_op = avoider.choose_action(observation_op, evaluate)
                action_target = searcher.choose_action(observation_target, evaluate)
                action = valuate_action(action_op, action_target, reward_op, reward_target)
                
                controller.robot_vel_publish(action)
                #rate.sleep()
                
                
                if(count % 200 == 0):
                    operator_vel = [ round(uniform(-0.3, 0.3), 1) for _ in range(3) ]
                    controller.operator_vel_publish(operator_vel)
                    #rate.sleep()
                    
                check_operator_limit(controller, operator_limit, rate)
                
                observation_op_, observation_target_ = observe(controller)
                
                reward_op = give_reward_opAvoidance(controller, action_op, old_action)
                reward_target = give_reward_targetSearcher(controller,
                                                           action_target)
                
                old_action = action_op

                done = check_target(controller)
                
                """
                agent.remember(observation, action, reward, observation_, done)
                
                if not evaluate and count % 50 == 0:
                    agent.learn()
                """
                
                observation_op = observation_op_
                observation_target = observation_target_
                
                print(count)
                #print(action)
                if(count == limit_count):
                    break;
                
            score_history.append(score)
            avg_score = np.mean(score_history[-100:])
            if avg_score > best_score:
                best_score = avg_score
            """
            if not evaluate and not rospy.is_shutdown():
                agent.save_models(i)
            """
            print('Episode ', i, 'score %.1f' % score, 'avg score %.1f' % avg_score,
              '---------------')                
                
        
    except rospy.ROSInterruptException:
        pass
    
    
