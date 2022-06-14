#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Feb 23 22:43:39 2022

@author: mauri
"""

import rospy
from listener_classes import Controller
from random import randint, uniform, choice
import numpy as np
from ddpg_classes import Agent
import pandas as pd
import math
from utils import HTrans, compute_ur_jacobian
import shutil


def debugInfo(n):
    print(f'Ciao:) - {n}')
    
def readPositionFile(file):
    position = np.genfromtxt(file)
    return position
    
def save_score(score_history, file, append):
    if append:
        with open(file, "a") as file:
            np.savetxt(file, score_history)
    else:
        with open(file, "w") as file:
            np.savetxt(file, score_history)
            
def save_config(config_history, file, append):
    if append:
        with open(file, "a") as file:
            np.savetxt(file, config_history)
    else:
        with open(file, "w") as file:
            np.savetxt(file, config_history)

def observe(ctrl, action):
    EE_pos, EEvel = ctrl.EE_pos.copy(), ctrl.EE_vel.copy()
    #theta_joints = np.array(ctrl.robot_pos).copy()
    target_pos = np.array(ctrl.target_pos).copy()
    obs = []
    
    """
    R_mat_ur10e = np.array([[0, -1, 0],
                            [1, 0, 0],
                            [0, 0, 1]])
    t_vect_EE = [-0.6, 0.0, 0.0]
    #print(f'or = {ctrl.EE_orientation}')
    th = np.array(ctrl.robot_pos).copy()
    th_mat = np.matrix([ [th[i]] for i in range(6) ])
    T_EE = HTrans(th_mat, 0)
    EE_pos1 = np.matmul(R_mat_ur10e, 
                           [ T_EE[i,3] for i in range(3) ])
    #EE_pos1 = np.matmul(R_mat2, EE_pos1)
    EE_pos1 = [ EE_pos1[i]+t_vect_EE[i] for i in range(3) ]
    T_d = np.matrix(np.identity(4), copy=False)
    T_d[2,3] = 0.195
    T_tcp = T_EE * T_d
    TCP_pos = np.matmul(R_mat_ur10e,
                        [ T_tcp[i,3] for i in range(3) ])
    TCP_pos = [ TCP_pos[i]+t_vect_EE[i] for i in range(3) ]
    print(f'EE_pos_sim = {EE_pos}',
      f'EE_pos_T = {EE_pos1}',
      f'TCP_pos = {TCP_pos}', sep='\n')
    """
    """
    print(f'vel_sim = {EEvel}')
    J = compute_ur_jacobian(th)
    EEvel = np.dot(J, action)[0:3]
    EEvel = np.matmul(R_mat_ur10e, EEvel)
    print(f'vel_J = {EEvel}')
    """
    """
    T_d = np.matrix(np.identity(4), copy=False)
    T_d[2,3] = -0.195
    EE_pos.append(1.0)
    EE_pos = np.dot(T_d, EE_pos)
    EE_pos = [ EE_pos[0,i] for i in range(3) ]
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
    """
    for k in range(6):
        obs.append(theta_joints[k])
    """        
    return obs

def give_reward(d_history, ctrl, v, old_v):    
    """
    max_acceleration = 300 deg/s^2
    dt = 0.05 s
    max_a = max_dv/0.05 -> max_dv = 15 deg/s = 0.26 rad/s
    """
    
    objPos = np.array(ctrl.target_pos).copy()
    EEpos = ctrl.EE_pos.copy()
    """
    T_d = np.matrix(np.identity(4), copy=False)
    T_d[2,3] = -0.195
    EEpos.append(1.0)
    EEpos = np.dot(T_d, EEpos)
    EEpos = [ EEpos[0,i] for i in range(3) ]
    """
    robot_acc = np.array(ctrl.robot_acc).copy()
    acc_r = 5*np.linalg.norm(robot_acc[0:3]) #dopo 80k ep si passa da 0.5 a 4

    
    d = 0
    for i in range(np.size(objPos)):
        d = d + (objPos[i] - EEpos[i])**2
    d = np.sqrt(d)
    """
    max_dv = 0.26 #se effettivamente si usano i radianti, cosa da verificare
    dv = [ abs(v[i]-old_v[i]) for i in range(3) ]
    c = [ 1.5 if dv[i]>max_dv else 0 for i in range(3) ]
    r = sum( [ x*y for (x, y) in zip(c, dv) ] )
    """
    r2 = 15*np.linalg.norm(v) #dopo 80k ep si passa da 2 a 10
    
    table_r = 100 if EEpos[2] < 0.05 else 0 #reward per evitare che l'EE tocchi il tavolo
    
    a = 25 #prima era 20
    reward = - a*d - table_r # - r2 - acc_r - table_r
    #print(f'acc_r = {acc_r} \n r = {a*d} \n v_r = {r2}')
    d_history.append(d)
    #d_history non viene utilizzato, ma può tornare utile per visualizzare
    #lo storico delle distanze volta per volta

    return reward, d_history

def check_target_OLD(ctrl):
    #funzione per controllare se il risultato è stato raggiunto o meno:
    #per farlo, bisogna arrivare in prossimità del target (sfera bianca) con
    #una velocità inferiore o ugule a 0.1 m/s
    delta = 0.02  
    
    EEpos = ctrl.EE_pos.copy() #se possibile, si effettua sempre la copia dei valori della classe controller
    objPos = np.array(ctrl.target_pos).copy()
    finalLinks = ctrl.finalLinks_spatialPos.copy()
    joints_spatialPos = ctrl.joints_spatialPos
    """
    d = 0
    for i in range(np.size(objPos)):
        d = d + (objPos[i] - EEpos[i])**2
    d = np.sqrt(d)
    """
    EE_angles = [finalLinks[0][0] - finalLinks[1][0], finalLinks[0][1] - finalLinks[1][1]]
    z_check = finalLinks[1][2] < finalLinks[0][2]

    check_bools_pos = [ objPos[i] <= EEpos[i]+0.05 and objPos[i] >= EEpos[i]-0.05 
                   for i in range(2) ]
    check_bools_pos.append( objPos[2] <= EEpos[2] and objPos[2] >= EEpos[2]-0.05 )
    check_angles = [ EE_angles[i]>=-delta and EE_angles[i]<=delta
                    for i in range(2) ]
    check_EE_pos = joints_spatialPos[4][0] > joints_spatialPos[3][0] #in questo modo l'EE non è girato ma rivolto in avanti
    
    check_bools = np.append(check_bools_pos, check_angles)
    check_bools = np.append(check_bools, z_check)
    check_bools = np.append(check_bools, check_EE_pos)

    check = True
    for i in range(np.size(check_bools)):
        check = check and check_bools[i]
    return check

def check_target(ctrl, count):
    #funzione per controllare se il risultato è stato raggiunto o meno:
    #per farlo, bisogna arrivare in prossimità del target (sfera bianca) con
    #una velocità inferiore o ugule a 0.1 m/s
    delta = 0.02 
    d_goal = 0.02
    
    """
    if count>=15_000 and count<=30_000:
        d_goal = 0.02
    elif count>=30_001: # and count<=2000:
        d_goal = 0.01
    """
    """
    elif count>=2001 and count<=3000:
        d_goal = 0.03
    elif count>=3001 and count<=4000:
        d_goal = 0.02
    elif count>=4001:
        d_goal = 0.01
    """
    EEpos = ctrl.EE_pos.copy() #se possibile, si effettua sempre la copia dei valori della classe controller
    """
    T_d = np.matrix(np.identity(4), copy=False)
    T_d[2,3] = -0.195
    #print(f'EE_pos = {EEpos}')
    EEpos.append(1.0)
    EEpos = np.dot(T_d, EEpos)
    EEpos = [ EEpos[0,i] for i in range(3) ]
    #print(f'tcp_pos = {EEpos}')
    """
    objPos = np.array(ctrl.target_pos).copy()
    finalLinks = ctrl.finalLinks_spatialPos.copy()
    joints_spatialPos = ctrl.joints_spatialPos
    
    d = 0
    for i in range(np.size(objPos)):
        d = d + (objPos[i] - EEpos[i])**2
    d = np.sqrt(d)
    
    EE_angles = [finalLinks[0][0] - finalLinks[1][0], finalLinks[0][1] - finalLinks[1][1]]
    z_check = finalLinks[1][2] < finalLinks[0][2]

    check_bools_pos = d <= d_goal and EEpos[2] >= objPos[2]
    check_angles = [ EE_angles[i]>=-delta and EE_angles[i]<=delta
                    for i in range(2) ]
    check_EE_pos = joints_spatialPos[4][0] > joints_spatialPos[3][0] #in questo modo l'EE non è girato ma rivolto in avanti
    
    check_bools = np.append(check_bools_pos, check_angles)
    check_bools = np.append(check_bools, z_check)
    check_bools = np.append(check_bools, check_EE_pos)

    return np.array(check_bools).all()


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
        
#Inserire anche V5 come V4 ??

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

        
        #######################
        #init DDPG stuff
        #######################
        
        append_data = False
        load_checkpoint = True #se True, carica i pesi e la memoria salvati
        evaluate = False #se True, non effettua il learn né il salvataggio dei dati
        select_remember = False #se vero, permette di salvare in memoria solo determinate transizioni
        changeInitialPosition = False
        
        cancel_rememberIteration = True #se True, consente di salvare sempre tutte le iterazioni, anche quando non avviene correttamente il reset
        avoidMemory = False
        
        reduction_factor = 0.2 #fattore di riduzione delle velocità del robot
                
        #meglio inserire anche theta_joints, per mantenere la proprietà di Markov                         
        observation_shape = (12,)  # [target_x-EEx, target_y-EEy, target_z-EEz,
                                  #  EE_vx, EE_vy, EE_vz]  
        
        #start at 0.1
        noise = 0.0
              
        agent = Agent(input_dims=observation_shape, n_actions=3, noise=noise,
                      chkpt_dir='tmp/ddpg_simplerSearcher_newEnv',
                      memory_dir='tmp/memory_simplerSearcher_newEnv',
                      reduction_factor=reduction_factor)
        #l'uscita servirà a controllare solo i primi 3 gradi di libertà,
        #mentre gli altri saranno impostati diversamente
        
        memory_file = 'tmp/score/score_memory_simplerSearcher_newEnv.csv'
        configuration_file = 'tmp/score/configuration_simplerSearcher_newEnv.csv'
        #Il file delle configurazioni serve per salvare le configurazioni in
        #cui si è trovato il robot (posizione dei giunti) al momento della raggiunta
        #del goal: servono per addestrare il bill avoider
        
        position_file = 'tmp/score/configuration_simplerSearcher_newEnv.csv'
        startPosition = readPositionFile(position_file)
        
        print('HAI CAMBIATO IL LOAD?',
              'HAI CAMBIATO IL NOISE?',
              'HAI CAMBIATO EVALUATE?',
              'HAI CAMBIATO IL SELECT REMEMBER?',
              'HAI CAMBIATO MEMORY FILE?',
              sep='\n')
        w = 0
        while(w<3000):
            w += 1
        
        best_score = 0
        n_games = 40_000
        n_games += 1 #per far si che al penultimo game si salvi la memoria (viene salvata ogn 100 episode, per non rallentare troppo il processo)
        limit_count = 1500 #numero di iterazioni massime per episode
        score_history = []
        success_history = []
        config_history = []
    
        loadMemory = evaluate or avoidMemory
    
        #routine di caricamento dei pesi e della memoria
        #per poter caricare i dati è necessario prima inizializzare la rete,
        #perciò si effettuano un numero di iterazioni casuali pari a batch_size
        #e poi si richiama la funzione learn: una volta fatto è poi possibile
        #sovrascrivere sia i pesi che la memoria con i dati salvati
        if load_checkpoint:
            n_steps = 0
            while n_steps <= agent.batch_size:
                observation = observe(controller, np.zeros(6))
                action = [ randint(1, 10)/10 for _ in range(3) ]
                #controller.robot_vel_publish(action)
                #rate.sleep()
                observation_ = observe(controller, np.zeros(6))
                reward, _ = give_reward([], controller, action, np.zeros(3))
                done = False
                agent.remember(observation, action, reward, observation_, done)
                n_steps += 1
            agent.learn()
            agent.my_load_models(evaluate=loadMemory)
            print('Loading completed:)')
        
        
        #PER CANCELLARE LA MEMORIA
        """
        input_shape = (12,)        
        agent.memory.state_memory = np.zeros((1_000_000, *input_shape)) #vettore degli stati presenti
        agent.memory.new_state_memory = np.zeros((1_000_000, *input_shape)) #vettore degli stati futuri
        agent.memory.action_memory = np.zeros((1_000_000, 3)) #vettore delle azioni compiute
        agent.memory.reward_memory = np.zeros(1_000_000) #vettore dei reward ottenuti
        agent.memory.terminal_memory = np.zeros(1_000_000, dtype=np.bool)
        """
        
        pick = False
        
        target_x = 0.0
        target_y = 0.0
        target_z = 0.0
        object_position = [0.0, 0.0, 0.0]
        
        max_x = 0.4 #0.5
        max_y = 0.4
        max_z = 0.4
        
        pick_conf = reset_pos_robot
        
        pick_vector = [[0.3, 0.409, 0.042],
                       [0.35, 0.301, 0.035]]
        place_vector = [[0.30, -0.014, 0.03],
                        [0.16, -0.014, 0.16]]
        
        src = [ 'tmp/ddpg_simplerSearcher_newEnv/actor_ddpg.h5',
                'tmp/ddpg_simplerSearcher_newEnv/critic_ddpg.h5',
                'tmp/ddpg_simplerSearcher_newEnv/target_actor_ddpg.h5',
                'tmp/ddpg_simplerSearcher_newEnv/target_critic_ddpg.h5' ]
        
        #routine di training/evalutation
        for ep in range(n_games):            
            """
            if ep == 14999:
                dst =  'tmp/checkpoint_tcp_searcher/EE_d6'
                for s in src:
                    shutil.copy(s, dst)
            elif ep == 29999:
                dst =  'tmp/checkpoint_tcp_searcher/EE_d3'
                for s in src:
                    shutil.copy(s, dst)
            
            elif ep == 24000:
                dst =  'tmp/checkpoint_tcp_searcher/d3'
                for s in src:
                    shutil.copy(s, dst)
            elif ep == 26000:
                dst =  'tmp/checkpoint_tcp_searcher/d2'
                for s in src:
                    shutil.copy(s, dst)
            """   
            
            if(rospy.is_shutdown()):
                break
            
            pick = not pick
            
            #Reset Routine
            controller.robot_vel_publish(start_vel_robot)
            #rate.sleep()
            print(pick)
            #controller.table_publish(resetTablePos)
            #rate.sleep()
            if pick:
                controller.robot_pos_publish()
            #rate.sleep()
            resetCompleted = False
            remember_iteration = True #se il sistema è bloccato, la successiva iterazione non verrà salvata
            c = 0
            #si attende che il robot torni nella posizione iniziale prima di continuare;
            #se entro 500000 iterazioni il posizionamento non è stato completato
            #(e, quindi, il robot è quasi sicuramente bloccato), allora si continua
            #comunque
            
            if pick:
                while(not resetCompleted):
                    r_pos = np.array(controller.robot_pos).copy() #robot position
                    resetCompleted = np.array(
                        [ r_pos[i]>=reset_pos_robot[i]-0.1 and r_pos[i]<=reset_pos_robot[i]+0.1 for i in range(6) ]
                        ).all()
                    c += 1
                    if(c == 500_000):
                        break;
                if(resetCompleted):
                    remember_iteration = True
                else:
                    remember_iteration = False
            else:
                while(not resetCompleted):
                    r_pos = np.array(controller.robot_pos).copy() #robot position
                    resetCompleted = np.array(
                        [ r_pos[i]>=pick_conf[i]-0.1 and r_pos[i]<=pick_conf[i]+0.1 for i in range(6) ]
                        ).all()
                    c += 1
                    if(c == 500_000):
                        break;
                if(resetCompleted):
                    remember_iteration = True
                else:
                    remember_iteration = False
            
            """
            if ep >= 30_000:
                changeInitialPosition = True
            """   
            change = choice([0,1])
            
            if changeInitialPosition and change and pick:
                startConfig = startPosition[choice(range(len(startPosition)))]
                controller.robot_pos_publish(startConfig, False)
                setConfigCompleted = False
                c = 0
                while(not setConfigCompleted):
                    r_pos = np.array(controller.robot_pos).copy()
                    setConfigCompleted = np.array(
                            [ r_pos[i]>=startConfig[i]-0.1 and r_pos[i]<=startConfig[i]+0.1 for i in range(6) ]
                            ).all()
                    c += 1
                    if(c == 500_000):
                        break;
                
            """
            target_x = round(uniform(0.5, 1.0), 2)
            target_y = round(uniform(-0.75, 0.75), 2)
            target_z = round(uniform(0.41, 0.7), 2)
            #target_z = 0.41
            """
            
            sign = lambda x: math.copysign(1, x)
            
            print(f'OLD = {object_position}')
            #New Env (senza tavolo)
            if pick:
                target_x = round(uniform(0.1, 0.40), 2) #round(uniform(0.0, 0.50), 2)
                target_y = round(uniform(-0.4, 0.4), 2) #round(uniform(-0.4, 0.4), 2)
                target_z = 0.20
            else:
                target_x = target_x + round(uniform(0.15, 0.30), 2) #round(uniform(0.20, 0.45), 2)
                target_y = target_y + round(uniform(0.0, 0.2), 2)*sign(target_y) #(0.0, 0.2)
                target_z = target_z + round(uniform(0.0, 0.2), 2)
            object_position = [ target_x if target_x<max_x else max_x,
                                target_y if abs(target_y)<max_y else max_y*sign(target_y),
                                target_z if target_z<max_z else max_z ]
            """
            if pick:
                object_position = choice(pick_vector)
            else:
                object_position = choice(place_vector)
            """
            
            print(f'NEW = {object_position}')
            controller.target_pos_publish(object_position)
            rate.sleep()
            rate.sleep()
            rate.sleep()
            
            observation = observe(controller, np.zeros(6)) #osservazione dello stato presente
            
            #reset variabili
            d_history = [] #storico delle distanze EE-target ad ogni iterazione
            done = False #terminal flag
            score = 0.0 #punteggio totale dell'episode
            
            count = 0 #numero delle iterazioni dell'episode
            
            old_reward = -1_000_000_000 #reward del ciclo precedente, utilizzato per selezionare quali iterazioni salvare e quali no
            old_action = np.zeros(3) 
            
            #ciclo effettivo dell'episode
            while not done:
                if(rospy.is_shutdown()):
                    print('ROS is shutdown:(')
                    break;
                
                count += 1    
                
                v = np.zeros(6)
                action = agent.choose_action(observation, evaluate) #calcolo dell'azione
                for ii in range(3):
                    v[ii] = action[ii]
                v[3] = findV4(controller)                
                v[4] = findV5(controller)
                
                controller.robot_vel_publish(v) #invio dell'azione a CoppeliaSim
                #rate.sleep()
                
                observation_ = observe(controller, v) #osservazione dello stato futuro, conseguente all'azione
                reward, d_history = give_reward(d_history, controller,
                                                action, old_action) #calcolo del reward
                done = check_target(controller, ep) #valutazione se il target è stato raggiunto o meno
                
                score += reward #aggiornamento dello score dell'episode
             
                if remember_iteration or cancel_rememberIteration:
                    if select_remember:
                        if(done==1 or reward>old_reward):
                            #il salvataggio in memoria si effettua solo se il punteggio aumenta rispetto all'iterazione precedente o se si è raggiunto il target
                            agent.remember(observation, action, reward, observation_, done)
                    else:
                        agent.remember(observation, action, reward, observation_, done)    
                
                old_reward = reward
                old_action = action
                
                if not evaluate and count % 50 == 0:
                    #ogni 50 iterazioni si effettua il learning, per non rallentare troppo la simulazione
                    agent.learn()
                
                #observation = observation_ #lo stato futuro diventa quello presente
                
                observation = list(np.array(observation_).copy())
                
                print(count)
                if(count == limit_count):
                    controller.robot_vel_publish(start_vel_robot)
                    break;
            
            controller.robot_vel_publish(start_vel_robot)
            if remember_iteration:    
                score_history.append(score)
                success_history.append(int(done))
            if(len(score_history) <= 1):
                avg_score = np.nan
            else:
                avg_score = np.mean(score_history[-100:]) #punteggio medio degli ultimi 100 episodi
            if avg_score > best_score:
                best_score = avg_score #indicazione della migliore media raggiunta
            if not evaluate and not rospy.is_shutdown():
                controller.robot_vel_publish(start_vel_robot)
                agent.save_models(ep)
            if done:
                controller.robot_vel_publish(start_vel_robot)
                config_history.append(controller.robot_pos)
                save_config(config_history, configuration_file, append_data)
                if append_data:
                    config_history = []
            if(ep%100==0 and ep!=0):
                save_score(score_history, memory_file, append_data)
                if append_data:
                    score_history = []
            print('Episode ', ep, 'score %.1f' % score, 'avg score %.1f' % avg_score,
              '---------------')
            if pick:
                if done:
                    pick_conf = controller.robot_pos
                else:
                    pick_conf = reset_pos_robot
        
        score_df = pd.DataFrame(list(zip(success_history, score_history)),
                                     columns=['success', 'score'])
        score_df.to_csv('tmp/score/score_dataFrame_searcher_newEnv.csv')
        
    except rospy.ROSInterruptException:
        pass
    
    
