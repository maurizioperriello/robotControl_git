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
    
def save_score(score_history, file, append):
    if append:
        with open(file, "a") as file:
            np.savetxt(file, score_history)
    else:
        with open(file, "w") as file:
            np.savetxt(file, score_history)

def observe_OLD(ctrl):
    #funzione per ottenere dalla classe del controller le informazioni utili
    #per conoscere l'ambiente: posizione dell'EE, velocità dell'EE
    # e posizione del target
    EEpos, EEvel = ctrl.EE_pos.copy(), ctrl.EE_vel.copy()
    finalLinks = ctrl.finalLinks_spatialPos.copy()
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
            
    EE_alpha = finalLinks[0][0] - finalLinks[1][0]
    EE_beta = finalLinks[0][1] - finalLinks[1][1]
    
    obs.append(EE_alpha)
    obs.append(EE_beta)
    
    for j in range(3):
        obs.append(target_pos[j])
            
    return obs

def observe(ctrl):
    EE_pos, EEvel = ctrl.EE_pos.copy(), ctrl.EE_vel.copy()
    theta_joints = np.array(ctrl.robot_pos).copy()
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

    for k in range(6):
        obs.append(theta_joints[k])
            
    return obs

def give_reward(d_history, ctrl):
    #reward: viene dato sulla base della distanza dell'EE dal target ed è
    #sempre negativo
    #Il coefficiente "a" viene abbassato nel caso in cui l'EE sia in vicinanza
    #del target con basse velocità: questo per aiutare il robot ad arrivare in 
    #prossimità di questo lentamente, per conseguire l'obiettivo di afferrare 
    #l'oggetto
    #Se EE_alpha e EE_beta sono entrambi zero, allora l'EE è allineato
    #verticalmente col terzultimo link (vedi CoppeliaSim) e, perciò, l'ultima
    #parte del robot è verticale
    delta = 0.02
    speed_limit = 0.1
    
    objPos = np.array(ctrl.target_pos).copy()
    EEpos, EEvel = ctrl.EE_pos.copy(), ctrl.EE_vel.copy()
    finalLinks = ctrl.finalLinks_spatialPos.copy()
    EE_alpha = finalLinks[0][0] - finalLinks[1][0] #differenza tra le coordinate x del terzultimo link e dell'EE
    EE_beta = finalLinks[0][1] - finalLinks[1][1] #differenza tra le coordinate y del terzultimo link e dell'EE
    z_check = finalLinks[1][2] < finalLinks[0][2] #in questo modo l'EE si trova girato verso il basso
    
    d = 0
    for i in range(np.size(objPos)):
        d = d + (objPos[i] - EEpos[i])**2
    d = np.sqrt(d)

    a = 1000
    if(d<=0.5):
        a = 500
        if(d<=0.3):
            a = 200
            if(EE_alpha>=-delta and EE_alpha<=delta and 
               EE_beta>=-delta and EE_beta<=delta and z_check):
                a = 50
                if(EEvel[0]>=-speed_limit and EEvel[1]>=-speed_limit and EEvel[2]>=-speed_limit and
                   EEvel[0]<=speed_limit and EEvel[1]<=speed_limit and EEvel[2]<=speed_limit):
                    a = 1

    reward = -a*d

    """
    a = 1000
    if(d<=0.5):
        a = 500
        if(d<=0.3):
            a = 200
            if(EEvel[0]>=-speed_limit and EEvel[1]>=-speed_limit and EEvel[2]>=-speed_limit and
               EEvel[0]<=speed_limit and EEvel[1]<=speed_limit and EEvel[2]<=speed_limit):
                a = 1
       
    b = 10
    if(EE_alpha>=-delta and EE_alpha<=delta and 
       EE_beta>=-delta and EE_beta<=delta and z_check):
                b = 0

    reward = - a*d - b*(abs(EE_alpha) + abs(EE_beta))
    """

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
    EE_angles = [finalLinks[0][0] - finalLinks[1][0], finalLinks[0][1] - finalLinks[1][1]]
    z_check = finalLinks[1][2] < finalLinks[0][2]
    check_bools_pos = [ objPos[i] <= EEpos[i]+0.15 and objPos[i] >= EEpos[i]-0.15 
                   for i in range(3) ]
    check_bools_vel = [ EEvel[i] <= 0.1 and EEvel[i] >= -0.1
                   for i in range(3) ]
    check_angles = [ EE_angles[i]>=-delta and EE_angles[i]<=delta
                    for i in range(2) ]
    check_bools = np.append(check_bools_pos, check_bools_vel)
    check_bools = np.append(check_bools, check_angles)
    check_bools = np.append(check_bools, z_check)

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
        #save_model = True
        evaluate = False #se True, non effettua il learn né il salvataggio dei dati
        select_remember = False #se vero, permette di salvare in memoria solo determinate transizioni
        
        
        #observation_shape = (11,)  # [EE_px, EE_py, EE_pz,
                                  #  EE_vx, EE_vy, EE_vz,
                                  #  EE_alpha, EE_beta, <- inclinazione ultimo link rispetto agli assi x e y
                                  #  target_x, target_y, target_z]
                                  
        observation_shape = (12,)  # [target_x, target_y, target_z,
                                  #  EE_vx, EE_vy, EE_vz,
                                  #  theta_i <- posizioni angolari dei 6 giunti
                                  #  ]    
        
        noise = 0.01
        #start at 0.1
        
        agent = Agent(input_dims=observation_shape, n_actions=6, noise=noise,
                      chkpt_dir='tmp/ddpg_targetHunter',
                      memory_dir='tmp/memory_searcher')
        
        memory_file = 'tmp/score_memory_targetHunter.csv'
        
        
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
        
    
        evaluate = False #se True, non effettua il learn né il salvataggio dei dati
        
        select_remember = False #se vero, permette di salvare in memoria solo determinate transizioni
        
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
               
            """
            target_x = round(uniform(0.2, 1.2), 2)
            if(target_x>=0.2 and target_x<=0.6):
                target_y = round(uniform(0.6, 1.2), 2)
                target_y *= choice([-1,1])
            else:
                target_y = round(uniform(0.0, 1.2), 2)
                target_y *= choice([-1,1])
            target_z = 0.41
            object_position = [target_x, target_y, target_z]
            """
            
            """
            object_position = [ round(uniform(0.5, 1.0), 2) for _ in range(2)]
            object_position[1] = object_position[1] * choice([-1,1])
            object_position.append(0.1)
            """
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
                
                action = agent.choose_action(observation, evaluate) #calcolo dell'azione
                controller.robot_vel_publish(action) #invio dell'azione a CoppeliaSim
                #rate.sleep()
                
                observation_ = observe(controller) #osservazione dello stato futuro, conseguente all'azione
                reward, d_history = give_reward(d_history, controller) #calcolo del reward
                done = check_target(controller) #valutazione se il target è stato raggiunto o meno
                
                score += reward #aggiornamento dello score dell'episode
                
                #TODO: da sistemare la memorizzazione selettiva delle iterazioni sulla base dei reward
                #if( (done==1 or reward>old_reward) and select_remember):
                    #il salvataggio in memoria si effettua solo se il punteggio aumenta rispetto all'iterazione precedente o se si è raggiunto il target
                if remember_iteration:
                    agent.remember(observation, action, reward, observation_, done) #salvataggio in menoria del dati necessari per il training
                old_reward = reward
                
                if not evaluate and count % 50 == 0:
                    #ogni 50 iterazioni si effettua il learning, per non rallentare troppo la simulazione
                    agent.learn()
                
                observation = observation_ #lo stato futuro diventa quello presente
                
                print(count)
                #print(action)
                #print(observation)
                if(count == limit_count):
                    break;
                
            score_history.append(score)
            avg_score = np.mean(score_history[-100:]) #punteggio medio degli ultimi 100 episodi
            if avg_score > best_score:
                best_score = avg_score #indicazione della migliore media raggiunta
            if not evaluate and not rospy.is_shutdown():
                controller.robot_vel_publish(start_vel_robot)
                agent.save_models(i)
            if(i%100==0 and i!=0):
                save_score(score_history, memory_file, append_data)
                if append_data:
                    score_history = []
            print('Episode ', i, 'score %.1f' % score, 'avg score %.1f' % avg_score,
              '---------------')                
            
        
    except rospy.ROSInterruptException:
        pass
    
    
