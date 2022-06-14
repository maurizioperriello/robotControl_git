#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu May 19 17:02:49 2022

@author: mauri
"""

import rospy
import numpy as np
from time import sleep
from AI_real_class import AI

#Filter AI
action_cntr = 0
speedVect_size = 22
n_filteredAction = 3
speed_vect = [ [ 0.0 for _ in range(speedVect_size) ] 
                           for c in range(n_filteredAction) ]

def speed_filter(action):
    global action_cntr
    global speed_vect
    index = action_cntr % speedVect_size
    for i in range(n_filteredAction):
        speed_vect[i][index] = action[i]
    action_cntr += 1
    return [ np.mean(speed_vect[i]) for i in range(n_filteredAction) ]
    
def reset_speed_filter():
    global action_cntr
    global speed_vect
    action_cntr = 0
    speed_vect = [ [ 0.0 for _ in range(speedVect_size) ] 
                           for c in range(n_filteredAction) ]
    
#Filter wrist
action_cntr_wrist = 0
speedVect_size_wrist = 1
speed_vect_wrist = [ [ 0.0 for _ in range(speedVect_size_wrist) ] 
                           for c in range(3) ]

def speed_filter_wrist(action):
    global action_cntr_wrist
    global speed_vect_wrist
    index = action_cntr_wrist % speedVect_size_wrist
    for i in range(3):
        speed_vect_wrist[i][index] = action[i]
    action_cntr_wrist += 1
    return [ np.mean(speed_vect_wrist[i]) for i in range(3) ]
    
def reset_speed_filter_wrist():
    global action_cntr_wrist
    global speed_vect_wrist
    action_cntr_wrist = 0
    speed_vect_wrist = [ [ 0.0 for _ in range(speedVect_size) ] 
                           for c in range(3) ]
    

if __name__ == '__main__':
    try:
        """
        project = { 'cube' : list_cubeName,
                    'position' : list_cubeFinalPos,
                    'priorities' : list_listPrecedenze,
                    'completed' : list_boolCubeIsInFinalPos}
        """
        """
        project = { 'cube' : ['red', 'green', 'blue', 'yellow', 'grey', 'sphere'],
                    'position' : [ [0.25,-0.25,0.05], [0.25,-0.25,0.135],
                                   [0.25,-0.05,0.05], [0.25,-0.05,0.135],
                                   [0.25,-0.15,0.195], [0.25,-0.15,0.256] ],
                    'priorities' : [ [], ['red'], [], ['blue'],
                                     ['red', 'green', 'blue', 'yellow'],
                                     ['red', 'green', 'blue', 'yellow', 'grey'] ],
                    'completed' : [ False for _ in range(6) ]}
        """
        
        dz = 0.05 + 0.195
        """
        project = { 'cube' : ['red', 'green', 'blue', 'yellow'],
                    'position' : [ [0.22, -0.014, 0.05+dz], [0.22, -0.014, 0.10+dz],
                                   [0.22, -0.014, 0.30+dz], [0.22, -0.014, 0.20+dz] ],
                    'priorities' : [ [], ['red', 'blue'], ['red'], ['red', 'blue', 'green']],
                    'completed' : [ False for _ in range(4) ]}
        """
        project = { 'cube' : ['red', 'green', 'blue', 'yellow'],
                    'position' : [ [0.30, -0.014, 0.20+dz], [0.16, -0.014, 0.16+dz],
                                   [0.10, -0.014, 0.40], [0.22, -0.014, 0.20] ],
                    'priorities' : [ [], ['red', 'blue'], [], ['red', 'blue', 'green']],
                    'completed' : [ False for _ in range(4) ]}
                    
                    #red_oldPos = [0.22, -0.014, 0.05+old_dz] old_new_z = 0.03
                    #green_oldPos = [0.16, -0.014, 0.12+old_dz]
                    #old_dz = 0.08 + 0.195
        
        """
        #PER USARE NUOVI BLOCCHI LEGO!!!
        project = { 'cube' : ['red', 'green', 'blue', 'yellow'],
                    'position' : [ [0.30, -0.014, 0.03+dz], [0.16, -0.014, 0.40],
                                   [0.16, -0.014, 0.03+dz], [0.22, -0.014, 0.20] ],
                    'priorities' : [ [], ['red', 'blue'], [], ['red', 'blue', 'green']],
                    'completed' : [ False for _ in range(4) ]}
        """
        agent = AI(project=project)
        
        episode_n = 1
        
        useGripper = True
        
        for ep in range(episode_n):  
            if(rospy.is_shutdown()):
                break
            agent.resetEnv()
            c = 0
            while(c<200_000):
                c += 1
            agent.scheduler_setUpTask()
            
            count = 0
            collision_counter = 0
            done = False
            doneRobot = False
            pick = False #True se l'oggetto è stato prelevato
            v = np.zeros(6)
            
            while 1:#(not done):
                if(rospy.is_shutdown()):
                    break
                
                bill_state = agent.getBillState()
                robot_state = agent.getRobotState()
                
                agent.checkBillTarget()
                doneRobot = agent.checkRobotTarget()

                v, collision = agent.doAction(v)
                if doneRobot:
                    print('Stop:)')
                    v = np.zeros(6)
                    reset_speed_filter()
                    reset_speed_filter_wrist()
                v[0:3] = speed_filter(v[0:3])
                #v[0:3] = np.zeros(3)
                agent.controller.robot_vel_publish(v)
                agent.rate_sleep()
                #print(agent.controller.redCube_pos)
                if doneRobot:
                    if useGripper:
                        agent.controller.commandGripper(pick, not pick)
                    else:
                        rospy.sleep(5)
                    pick = not pick
                    doneRobot = False
                    print('Restart:)')
                
                #Check target
                #agent.checkAllTarget()
                
                if collision:
                    #print('Collision:(')
                    collision_counter += 1
                
                done = agent.isProjectCompleted()
                count += 1
            
            #agent.resetBillAndUr10e()
            print(f'Total collision: {collision_counter}')
            print('------------------------',
                  f'End of episode {ep}',
                  '------------------------', sep='\n')
            #sleep(3)
            
    except rospy.ROSInterruptException:
        pass
