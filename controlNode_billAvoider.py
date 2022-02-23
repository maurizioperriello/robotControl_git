#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Feb 23 16:18:27 2022

@author: mauri
"""


import rospy
from listener_classes import Controller
from bill_controller import Bill
import numpy as np
from random import randint, uniform, choice
from ddpg_classes import Agent
from utils import pnt2line

def debugInfo(n):
    print(f'Ciao:) - {n}')

def observe(ctrl, bill):
    theta_joints = np.array(ctrl.robot_pos).copy()
    EE_vel = ctrl.EE_vel.copy()
    billLimbs = np.array(bill.spatialPose).copy()
    limbSelector = [4, 6, 7] #testa, mano destra, mano sinistra (in realtà un punto tra la mano e il gomito)
    obs = []
    
    for i in range(3):
        obs.append(theta_joints[i])
    for i in range(3):
        obs.append(EE_vel[i])
    for i in limbSelector:
        for j in range(3):
            obs.append(billLimbs[i][j])
            
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

def check_operator_collision(ctrl, op_ctrl):
    #RIDURRE IL NUMERO DI PARAGONI (USARE index=[2,4]?)
    spatialPos_joints_robot = ctrl.joints_spatialPos.copy()
    point_op = op_ctrl.spatialPose.copy()
    min_d = 0.3 #minima distanza entro la quale si considera una collisione
        
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
        

if __name__ == '__main__':
    try:
        #######################
        #init ROS stuff
        #######################        
        
        #Bill
        bill = Bill()
        
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
        
        
        #######################
        #init DDPG stuff
        #######################
        
        load_checkpoint = False
        evaluate = False
        
        #noise start at 0.1
        noise = 0.1
        
        observation_shape = (15,)
        #  [op_px, op_py, op_pz, op_vx, op_vy, op_vz,
        #   joint1_x, joint1_y, joint1_z,
        #   joint2_x, joint2_y, joint2_z,
        #   joint3_x, joint3_y, joint3_z,]    
        
        agent = Agent(input_dims=observation_shape, n_actions=6,
                      chkpt_dir='tmp/ddpg_billAvoidance',
                      memory_dir='tmp/memory_billAvoider', noise=noise)        
        
        best_score = 0
        n_games = 10_000
        n_games += 1
        limit_count = 8000
        score_history = []
        
        
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
            
            bill.vel_publishFun(np.zeros(10))
            #rate.sleep()
            bill.reset_publishFun()
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
                
                
                if(count % 200 == 0):
                    bill_vel = [ round(uniform(-0.2, 0.2), 2) for _ in range(10) ]
                    bill.vel_publishFun(bill_vel)
                    #rate.sleep()
                    
                #check_operator_limit(controller, operator_limit, rate)
                
                
                observation_ = observe(controller)
                reward = give_reward(controller, bill, action, old_action)
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
    
    


