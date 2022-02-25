#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Feb 23 22:43:39 2022

@author: mauri
"""

import rospy
from listener_classes import Controller
from random import randint, uniform
import numpy as np
from ddpg_classes import Agent



def debugInfo(n):
    print(f'Ciao:) - {n}')
    
def save_score(score_history, file, append):
    if append:
        with open(file, "a") as file:
            np.savetxt(file, score_history)
    else:
        with open(file, "w") as file:
            np.savetxt(file, score_history)
            

def observe(ctrl):
    EE_pos, EEvel = ctrl.EE_pos.copy(), ctrl.EE_vel.copy()
    #theta_joints = np.array(ctrl.robot_pos).copy()
    target_pos = np.array(ctrl.target_pos).copy()
    obs = []
    
    for j in range(3):
        obs.append(target_pos[j]-EE_pos[j])
      
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

def give_reward(d_history, ctrl):
    #reward: viene dato sulla base della distanza dell'EE dal target ed è
    #sempre negativo
    #Il coefficiente "a" viene abbassato nel caso in cui l'EE sia in vicinanza
    #del target con basse velocità: questo per aiutare il robot ad arrivare in 
    #prossimità di questo lentamente, per conseguire l'obiettivo di afferrare 
    #l'oggetto
    speed_limit = 0.1
    
    objPos = np.array(ctrl.target_pos).copy()
    EEpos, EEvel = ctrl.EE_pos.copy(), ctrl.EE_vel.copy()

    vel_check = EEvel[0]>=-speed_limit and EEvel[1]>=-speed_limit and EEvel[2]>=-speed_limit and \
        EEvel[0]<=speed_limit and EEvel[1]<=speed_limit and EEvel[2]<=speed_limit
    
    d = 0
    for i in range(np.size(objPos)):
        d = d + (objPos[i] - EEpos[i])**2
    d = np.sqrt(d)

    a = 500
    if(d<=0.55):
        a = 200
        if(d<=0.35):
            a = 50
            if(vel_check):
                a = 1
                
    reward = -a*d
    
    
    d_history.append(d)
    #d_history non viene utilizzato, ma può tornare utile per visualizzare
    #lo storico delle distanze volta per volta

    return reward, d_history


def check_target(ctrl):
    #funzione per controllare se il risultato è stato raggiunto o meno:
    #per farlo, bisogna arrivare in prossimità del target (sfera bianca) con
    #una velocità inferiore o ugule a 0.1 m/s
    delta = 0.02
    
    EEpos, EEvel = ctrl.EE_pos.copy(), ctrl.EE_vel.copy() #se possibile, si effettua sempre la copia dei valori della classe controller
    objPos = np.array(ctrl.target_pos).copy()
    finalLinks = ctrl.finalLinks_spatialPos.copy()
    joints_spatialPos = ctrl.joints_spatialPos
    
    EE_angles = [finalLinks[0][0] - finalLinks[1][0], finalLinks[0][1] - finalLinks[1][1]]
    z_check = finalLinks[1][2] < finalLinks[0][2]
    check_bools_pos = [ objPos[i] <= EEpos[i]+0.15 and objPos[i] >= EEpos[i]-0.15 
                   for i in range(3) ]
    check_bools_vel = [ EEvel[i] <= 0.1 and EEvel[i] >= -0.1
                   for i in range(3) ]
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
    k = 10
    finalLinks = ctrl.finalLinks_spatialPos.copy()
    EE_alpha = finalLinks[0][0] - finalLinks[1][0] #differenza tra le coordinate x del terzultimo link e dell'EE
    EE_beta = finalLinks[0][1] - finalLinks[1][1]
    d = np.sqrt(EE_alpha**2 + EE_beta**2)
    d1r = 1 if finalLinks[0][0]>finalLinks[1][0] else 0 #perché "dir" è meglio non usarlo
    v4 = k * d * (-1)**d1r
    return v4
        

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
        
        append_data = True
        load_checkpoint = True #se True, carica i pesi e la memoria salvati
        evaluate = False #se True, non effettua il learn né il salvataggio dei dati
        select_remember = False #se vero, permette di salvare in memoria solo determinate transizioni
        
        #meglio inserire anche theta_joints, per mantenere la proprietà di Markov                         
        observation_shape = (6,)  # [target_x-EEx, target_y-EEy, target_z-EEz,
                                  #  EE_vx, EE_vy, EE_vz]  
        
        noise = 0.1
        #start at 0.1
        
        agent = Agent(input_dims=observation_shape, n_actions=3, noise=noise,
                      chkpt_dir='tmp/ddpg_simplerSearcher',
                      memory_dir='tmp/memory_simplerSearcher')
        #l'uscita servirà a controllare solo i primi 3 gradi di libertà,
        #mentre gli altri saranno impostati diversamente
        
        memory_file = 'tmp/score_memory_simplerSearcher.csv'
        
        
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
        n_games = 8000
        n_games += 1 #per far si che al penultimo game si salvi la memoria (viene salvata ogn 100 episode, per non rallentare troppo il processo)
        limit_count = 3000 #numero di iterazioni massime per episode
        score_history = []
    
        #routine di caricamento dei pesi e della memoria
        #per poter caricare i dati è necessario prima inizializzare la rete,
        #perciò si effettuano un numero di iterazioni casuali pari a batch_size
        #e poi si richiama la funzione learn: una volta fatto è poi possibile
        #sovrascrivere sia i pesi che la memoria con i dati salvati
        if load_checkpoint:
            n_steps = 0
            while n_steps <= agent.batch_size:
                observation = observe(controller)
                action = [ randint(1, 10)/10 for _ in range(3) ]
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
        
        #routine di training/evalutation
        for ep in range(n_games):
            if(rospy.is_shutdown()):
                break
            
            #Reset Routine
            controller.robot_vel_publish(start_vel_robot)
            #rate.sleep()
            
            controller.table_publish(resetTablePos)
            #rate.sleep()
            controller.robot_pos_publish()
            #rate.sleep()
            resetCompleted = False
            remember_iteration = True #se il sistema è bloccato, la successiva iterazione non verrà salvata
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
            if(resetCompleted):
                remember_iteration = True
            else:
                remember_iteration = False
            controller.table_publish(startTablePos)
            #rate.sleep()
                    
            target_x = round(uniform(0.5, 1.0), 2)
            target_y = round(uniform(-0.75, 0.75), 2)
            target_z = 0.41
            object_position = [target_x, target_y, target_z]
            
            controller.target_pos_publish(object_position)
            #rate.sleep()
            
            observation = observe(controller) #osservazione dello stato presente
            
            #reset variabili
            d_history = [] #storico delle distanze EE-target ad ogni iterazione
            done = False #terminal flag
            score = 0.0 #punteggio totale dell'episode
            
            count = 0 #numero delle iterazione dell'episode
            
            old_reward = -1_000_000_000 #reward del ciclo precedente, utilizzato per selezionare quali iterazioni salvare e quali no
                        
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
                
                controller.robot_vel_publish(v) #invio dell'azione a CoppeliaSim
                #rate.sleep()
                
                observation_ = observe(controller) #osservazione dello stato futuro, conseguente all'azione
                reward, d_history = give_reward(d_history, controller) #calcolo del reward
                done = check_target(controller) #valutazione se il target è stato raggiunto o meno
                
                score += reward #aggiornamento dello score dell'episode
             
                if remember_iteration:
                    if select_remember:
                        if(done==1 or reward>old_reward):
                            #il salvataggio in memoria si effettua solo se il punteggio aumenta rispetto all'iterazione precedente o se si è raggiunto il target
                            agent.remember(observation, action, reward, observation_, done)
                    else:
                        agent.remember(observation, action, reward, observation_, done)    
                
                old_reward = reward
                
                if not evaluate and count % 50 == 0:
                    #ogni 50 iterazioni si effettua il learning, per non rallentare troppo la simulazione
                    agent.learn()
                
                observation = observation_ #lo stato futuro diventa quello presente
                
                print(count)
                if(count == limit_count):
                    break;
                
            score_history.append(score)
            avg_score = np.mean(score_history[-100:]) #punteggio medio degli ultimi 100 episodi
            if avg_score > best_score:
                best_score = avg_score #indicazione della migliore media raggiunta
            if not evaluate and not rospy.is_shutdown():
                controller.robot_vel_publish(start_vel_robot)
                agent.save_models(ep)
            if(ep%100==0 and ep!=0):
                save_score(score_history, memory_file, append_data)
                if append_data:
                    score_history = []
            print('Episode ', ep, 'score %.1f' % score, 'avg score %.1f' % avg_score,
              '---------------')                
            
        
    except rospy.ROSInterruptException:
        pass
    
    
