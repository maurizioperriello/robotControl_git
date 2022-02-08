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
    joints_spatialPos = ctrl.joints_spatialPos
    opPos = ctrl.operator_pos
    opVel = ctrl.operator_vel
    jointSelection = [1,2,3]
    
    obs = []
    
    for i in range(3):
        obs.append(opPos[i])
    for i in range(3):
        obs.append(opVel[i])
    for i in jointSelection:
        for j in range(3):
            obs.append(joints_spatialPos[i][j])
            
    return obs

def give_reward(ctrl, op_ctrl, v, oldv):
    dv_max = 0.7    
    reward = 0 
    selfCollisionRobot = ctrl.selfCollision_bool
    globalCollisionRobot = ctrl.globalCollision_bool
    
    mod_v = 0
    for i in range(len(v)):
        mod_v = mod_v + v[i]**2
    mod_v = np.sqrt(mod_v)
    collision, max_distance_bool = check_operator_collision(ctrl, op_ctrl)
    
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

def check_operator_collision_OLD(ctrl):
    #funzione per individuare se ci sono collisioni tra l'operatore (pallina verde) e il robot
    spatialPos_joints = ctrl.joints_spatialPos.copy()
    opPos = ctrl.operator_pos
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

def check_operator_collision(ctrl, op_ctrl):
    #RIDURRE IL NUMERO DI PARAGONI (USARE index=[2,4]?)
    spatialPos_joints_robot = ctrl.joints_spatialPos.copy()
    spatialPos_joints_op = op_ctrl.joints_spatialPos.copy()
    min_d = 0.3 #minima distanza entro la quale si considera una collisione
    point_op = []
    index = [0, 2, 4]
    for i in index:
        point_op.append(spatialPos_joints_op[i])
        midpoint = [ (spatialPos_joints_op[i][j] + spatialPos_joints_op[i+1][j])/2
                    for j in range(3) ]
        point_op.append(midpoint)
        point_op.append(spatialPos_joints_op[i+1])
        if(i != 4):
            midpoint = [ (spatialPos_joints_op[i+1][j] + spatialPos_joints_op[i+2][j])/2
                    for j in range(3) ]
            point_op.append(midpoint)
        
    l = len(point_op)
    check_d = [ False for _ in range(6*l) ]
    d_max = 0
    for k in range(l):
        d0, _ = pnt2line(point_op[k] , [0,0,0], spatialPos_joints_robot[0])
        if(d0 > d_max):
            d_max = d0
        ind = 6*k
        check_d[ind] = d0 <= min_d
        for i in range(5):
            d, _ = pnt2line(point_op[k], spatialPos_joints_robot[i], spatialPos_joints_robot[i+1])
            check_d[ind+i+1] = d <= min_d
            if(d > d_max):
                d_max = d
        
    #print(check_d)
    
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
        

if __name__ == '__main__':
    try:
        #######################
        #init ROS stuff
        #######################
        
        
        #Nome nodo
        nodeName = 'operator_look'
        
        #Publisher Topic
        robotPos_pubTop = 'topic1'
        robotVel_pubTop = 'topic2'
        targetPos_pubTop = 'topic3'
        opPos_pubTop = 'topic4'
        opVel_pubTop = 'topic5'
        resetRobot_pubTop = 'topic6'
        
        #Subscriber Topic
        op_subs = 'obstacle_op_pos'
        EEpos_subs = 'ee_pose_op'
        EEvel_subs = 'ee_velocity_op'
        targetPos_subs = 'target_pos'
        robot_subs = 'joints_value_op'
        spatialPosRobotJoints_subs = 'spatialPos_joints_op'
        selfCollisionRobot_subs = 'collision_operator'
        globalCollisionRobot_subs = 'global_operator_collision'
        
        operator_looker = Controller(nodeName, robotPos_pubTop, robotVel_pubTop, targetPos_pubTop,
                                opPos_pubTop, opVel_pubTop, resetRobot_pubTop,
                                op_subs, EEpos_subs, EEvel_subs, targetPos_subs,
                                robot_subs, spatialPosRobotJoints_subs, selfCollisionRobot_subs,
                                globalCollisionRobot_subs)
        
        
        #Nome nodo
        nodeName = 'robotControl'
        
        #Publisher Topic
        robotPos_pubTop = 'joints_pose'
        robotVel_pubTop = 'joints_velocity'
        targetPos_pubTop = 'sphere_pose'
        opPos_pubTop = 'op_pose'
        opVel_pubTop = 'obstacle_op_velocity'
        resetRobot_pubTop = 'reset_robot'
        
        #Subscriber Topic
        op_subs = 'obstacle_op_pos'
        EEpos_subs = 'ee_pose'
        EEvel_subs = 'ee_velocity'
        targetPos_subs = 'target_pos'
        robot_subs = 'joints_value'
        spatialPosRobotJoints_subs = 'spatialPos_joints'
        selfCollisionRobot_subs = 'collision_robot'
        globalCollisionRobot_subs = 'global_robot_collision'
        
        controller = Controller(nodeName, robotPos_pubTop, robotVel_pubTop, targetPos_pubTop,
                                opPos_pubTop, opVel_pubTop, resetRobot_pubTop,
                                op_subs, EEpos_subs, EEvel_subs, targetPos_subs,
                                robot_subs, spatialPosRobotJoints_subs, selfCollisionRobot_subs,
                                globalCollisionRobot_subs)
        rate = controller.rate
        
        reset_pos_robot = controller.start_pos_robot.copy()
        start_vel_robot = np.zeros(6)
        
        operator_limit = [[-0.7, 0.7], [-0.7, 0.7], [0.3, 0.7]]
        
        
        #######################
        #init DDPG stuff
        #######################
        
        observation_shape = (15,)
        #  [op_px, op_py, op_pz, op_vx, op_vy, op_vz,
        #   joint1_x, joint1_y, joint1_z,
        #   joint2_x, joint2_y, joint2_z,
        #   joint3_x, joint3_y, joint3_z,]    
        
        agent = Agent(input_dims=observation_shape, n_actions=6,
                      fc1=600, fc2=300,
                      chkpt_dir='tmp/ddpg_opAvoidance',
                      memory_dir='tmp/memory_aboider')        
        
        best_score = 0
        n_games = 10_000
        n_games += 1
        limit_count = 8000
        score_history = []
        load_checkpoint = False

        evaluate = True
        
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
                
                """
                if(count % 200 == 0):
                    operator_vel = [ round(uniform(-0.3, 0.3), 1) for _ in range(3) ]
                    controller.operator_vel_publish(operator_vel)
                    #rate.sleep()
                    
                check_operator_limit(controller, operator_limit, rate)
                """
                
                observation_ = observe(controller)
                reward = give_reward(controller, operator_looker, action, old_action)
                old_action = action

                done = False
                
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
                agent.save_models(i)
            print('Episode ', i, 'score %.1f' % score, 'avg score %.1f' % avg_score,
              '---------------')                
                
        
    except rospy.ROSInterruptException:
        pass
    
    
