#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu May 19 17:02:49 2022

@author: mauri
"""

import rospy
from time import sleep
from AI_real_class import AI

if __name__ == '__main__':
    try:
        """
        project = { 'cube' : list_cubeName,
                    'position' : list_cubeFinalPos,
                    'priorities' : list_listPrecedenze,
                    'completed' : list_boolCubeIsInFinalPos}
        """
        project = { 'cube' : ['red', 'green', 'blue', 'yellow', 'grey', 'sphere'],
                    'position' : [ [0.5,-0.3,0.05], [0.5,-0.3,0.135],
                                   [0.5,-0.1,0.05], [0.5,-0.1,0.135],
                                   [0.5,-0.2,0.195], [0.5,-0.2,0.256] ],
                    'priorities' : [ [], ['red'], [], ['blue'],
                                     ['red', 'green', 'blue', 'yellow'],
                                     ['red', 'green', 'blue', 'yellow', 'grey'] ],
                    'completed' : [ False for _ in range(6) ]}
        
        agent = AI(project=project)
        
        episode_n = 1
        
        for ep in range(episode_n):  
            if(rospy.is_shutdown()):
                break
            agent.resetEnv()
            c = 0
            while(c<200_000):
                c += 1
            agent.scheduler_setUpTask()
            #Load NN
            #agent.load_weights() #si fa già nel costruttore della classe
            #Set up task
            #agent.scheduler_setUpTask() #si fa già nel costruttore dello scheduler
            
            count = 0
            collision_counter = 0
            done = False
            
            while(not done):
                if(rospy.is_shutdown()):
                    break
                
                bill_state = agent.getBillState()
                robot_state = agent.getRobotState()

                collision = agent.doAction()
                #agent.rate_sleep()
                
                #Check target
                agent.checkAllTarget()
                
                if collision:
                    print('Collision:(')
                    collision_counter += 1
                
                done = agent.isProjectCompleted()
                count += 1
            
            agent.resetBillAndUr10e()
            print(f'Total collision: {collision_counter}')
            print('------------------------',
                  f'End of episode {ep}',
                  '------------------------', sep='\n')
            sleep(3)
            
    except rospy.ROSInterruptException:
        pass
