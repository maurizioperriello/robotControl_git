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

#Filter
action_cntr = 0
speedVect_size = 50
n_filteredAction = 6
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
        project = { 'cube' : ['red', 'green', 'blue', 'yellow'],
                    'position' : [ [0.20,-0.20,0.05], [0.20,-0.20,0.10],
                                   [0.20,-0.20,0.15], [0.20,-0.20,0.20] ],
                    'priorities' : [ [], ['red', 'blue'], ['red'], ['red', 'blue', 'green']],
                    'completed' : [ False for _ in range(6) ]}
        
        agent = AI(project=project)
        
        episode_n = 1
        
        useGripper = False
        
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
            
            while(not done):
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
                v = speed_filter(v)
                agent.controller.robot_vel_publish(v)
                agent.rate_sleep()
                
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
            sleep(3)
            
    except rospy.ROSInterruptException:
        pass
