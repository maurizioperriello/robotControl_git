#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Dec  3 09:23:26 2021

@author: mauri
"""

#######################################################################
#
#   Questo nodo si occupa di controllare un robot identico all'altro
#   che svolge il compito di operatore (questo è il robot rosso)
#
#######################################################################

import rospy
from listener_classes import Controller
import numpy as np
from random import randint, uniform, choice
from ddpg_classes import Agent
from utils import pnt2line

def debugInfo(n):
    print(f'Ciao:) - {n}')

def observe(ctrl):
    #funzione per ottenere dalla classe del controller le informazioni utili
    #per conoscere l'ambiente: posizione dell'EE, velocità dell'EE
    # e posizione del target
    EEpos, EEvel = ctrl.EE_pos.copy(), ctrl.EE_vel.copy()
    target_pos = np.array(ctrl.target_pos).copy()
    obs = []
        
    if(np.size(EEpos) == 3):
        for i in range(3):
            obs.append(EEpos[i])
    else:
        for i in range(3):
            obs.append(0.0)
            
    if(np.size(EEvel) == 3):
        for i in range(3):
            obs.append(EEvel[i])
    else:
        for i in range(3):
            obs.append(0.0)
    
    for j in range(3):
        obs.append(target_pos[j])
            
    return obs

def give_reward(d_history, ctrl):
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
        

if __name__ == '__main__':
    try:
        #######################
        #init ROS stuff
        #######################
        
        #Nome nodo
        nodeName = 'operatorControl'
        
        #Publisher Topic
        robotPos_pubTop = 'joints_pose_op'
        robotVel_pubTop = 'joints_velocity_op'
        targetPos_pubTop = 'sphere_pose'
        opPos_pubTop = 'op_pose'
        opVel_pubTop = 'obstacle_op_velocity'
        resetRobot_pubTop = 'reset_operator'
        
        #Subscriber Topic
        op_subs = 'obstacle_op_pos'
        EEpos_subs = 'ee_pose_op'
        EEvel_subs = 'ee_velocity_op'
        targetPos_subs = 'target_pos'
        robot_subs = 'joints_value_op'
        spatialPosRobotJoints_subs = 'spatialPos_joints_op'
        selfCollisionRobot_subs = 'collision_operator'
        globalCollisionRobot_subs = 'global_operator_collision'
        
        controller = Controller(nodeName, robotPos_pubTop, robotVel_pubTop, targetPos_pubTop,
                                opPos_pubTop, opVel_pubTop, resetRobot_pubTop,
                                op_subs, EEpos_subs, EEvel_subs, targetPos_subs,
                                robot_subs, spatialPosRobotJoints_subs, selfCollisionRobot_subs,
                                globalCollisionRobot_subs)
        rate = controller.rate #non viene utilizzata
        
        reset_pos_robot = controller.start_pos_robot.copy() #posizione iniziale del robot
        start_vel_robot = np.zeros(6) #velocità iniziale del robot      
        
        #######################
        #init DDPG stuff
        #######################
        
        observation_shape = (9,)  # [EE_px, EE_py, EE_pz, EE_vx, EE_vy, EE_vz, target_x, target_y, target_z]    
        
        agent = Agent(input_dims=observation_shape, n_actions=6,
                      chkpt_dir='tmp/ddpg_targetHunter',
                      memory_dir='tmp/memory_searcher')        
        
        best_score = 0
        n_games = 1_000_000
        n_games += 1 #per far si che al penultimo game si salvi la memoria (viene salvata ogn 100 episode, per non rallentare troppo il processo)
        limit_count = 3000 #numero di iterazioni massime per episode
        score_history = []
        load_checkpoint = True #se True, carica i pesi e la memoria salvati
        #save_model = True
        
        #creare routine per sviluppare l'evaluation
        evaluate = True #se True, non effettua il learn né il salvataggio dei dati
        
        #routine di caricamento dei pesi e della memoria
        #per poter caricare i dati è necessario prima inizializzare la rete,
        #perciò si effettuano un numero di iterazioni casuali pari a batch_size
        #e poi si richiama la funzione learn: una volta fatto è poi possibile
        #sovrascrivere sia i pesi che la memoria con i dati salvati
        if load_checkpoint:
            n_steps = 0
            while n_steps <= agent.batch_size:
                observation = observe(controller)
                action = [ randint(1, 10)/10 for _ in range(6) ]
                controller.robot_vel_publish(action)
                #rate.sleep()
                observation_ = observe(controller)
                reward, _ = give_reward([], controller)
                done = check_target(controller)
                agent.remember(observation, action, reward, observation_, done)
                n_steps += 1
            agent.learn()
            agent.my_load_models()
            print('Loading completed:)')
        
        #serie di print per verificare se effettivamente i dati sono stati
        #caricati correttamente
        print('------------------')
        print(agent.memory.state_memory[999_999],
              agent.memory.new_state_memory[999_999],
              agent.memory.action_memory[999_999],
              agent.memory.reward_memory[999_999],
              agent.memory.terminal_memory[999_999],
              sep='\n')
        print('------------------')
        
        #routine di training/evalutation
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
            #si attende che il robot torni nella posizione iniziale prima di continuare;
            #se entro 500000 iterazioni il posizionamento non è stato completato
            #(e, quindi, il robot è quasi sicuramente bloccato), allora si continua
            #comunque
            while(not resetCompleted):
                r_pos = np.array(controller.robot_pos).copy() #robot position
                resetCompleted = np.array(
                    [ r_pos[i]>=reset_pos_robot[i]-0.1 and r_pos[i]<=reset_pos_robot[i]+0.1 for i in range(6) ]
                    ).all()
                c += 1
                if(c == 500_000):
                    break;
            
            object_position = [ round(uniform(-0.5, 0.5), 2) for _ in range(2)]
            #object_position[1] = object_position[1] * choice([-1,1])
            object_position.append(0.1)
            controller.target_pos_publish(object_position)
            #rate.sleep()
            
            observation = observe(controller) #osservazione dello stato presente
            
            #reset variabili
            d_history = [] #storico delle distanze EE-target ad ogni iterazione
            done = False #terminal flag
            score = 0.0 #punteggio totale dell'episode
            
            count = 0 #numero delle iterazione dell'episode
                        
            #ciclo effettivo dell'episode
            while not done:
                if(rospy.is_shutdown()):
                    print('ROS is shutdown:(')
                    break;
                
                count += 1                
                
                action = agent.choose_action(observation, evaluate) #calcolo dell'azione
                controller.robot_vel_publish(action) #invio dell'azione a CoppeliaSim
                #rate.sleep()
                
                observation_ = observe(controller) #osservazione dello stato futuro, conseguente all'azione
                reward, d_history = give_reward(d_history, controller) #calcolo del reward
                done = check_target(controller) #valutazione se il target è stato raggiunto o meno
                
                score += reward #aggiornamento dello score dell'episode
                
                #agent.remember(observation, action, reward, observation_, done) #salvataggio in menoria del dati necessari per il training
                """
                if not evaluate and count % 50 == 0:
                    #ogni 50 iterazioni si effettua il learning, per non rallentare troppo la simulazione
                    agent.learn()
                """
                
                observation = observation_ #lo stato futuro diventa quello presente
                
                print(count)
                #print(action)
                if(count == limit_count):
                    break;
                
            score_history.append(score)
            avg_score = np.mean(score_history[-100:]) #punteggio medio degli ultimi 100 episodi
            if avg_score > best_score:
                best_score = avg_score #indicazione della migliore media raggiunta
            #if not evaluate and not rospy.is_shutdown():
                #agent.save_models(i)
            print('Episode ', i, 'score %.1f' % score, 'avg score %.1f' % avg_score,
              '---------------')                
                
        
    except rospy.ROSInterruptException:
        pass
    
    
