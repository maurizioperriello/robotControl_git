#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu May 19 14:54:20 2022

@author: mauri
"""

import numpy as np
from numpy import tanh
from ddpg_classes import Agent
from controller_real_class import Controller
from utils import pnt2line, AH


class Scheduler:
    def __init__(self, project, ctrl):
        """
        project = { 'cube' : list_cubeName,
                    'position' : list_cubeFinalPos,
                    'priorities' : list_listPrecedenze,
                    'completed' : list_boolCubeIsInFinalPos}
        """
        self.project = project
        self.scController = ctrl
        
        #Tasks
        """
        task = { 'cube' : list_cubeName,
                 'position' : list_listStartPosFinalPos,
                 'completed' : list_listBoolCompletedAction}
        task_index = [index_generalTask, index_specificAction]
        
        index_generalTask = indica la task da eseguire, può andare da 0 a 2
        index_specificAction = indica l'azione specifica da eseguire all'interno
                                della task in corso, ossia se si deve prelevare
                                l'oggetto oppure posizionarlo, 0 nel primo caso e 
                                1 nel secondo
        """
        self.max_task = 3
        self.robot_tasks = None
        self.bill_tasks = None
        self.robot_task_ind = None
        self.bill_task_ind = None
        
        self.robotFinish = False
        self.billFinish = False

         
    def changeProject(self, project):
        self.project = project
        
    def clearRobotTask(self):
        self.robot_tasks['completed'][self.robot_task_ind[0]][self.robot_task_ind[1]] = True
        
    def clearBillTask(self):
        self.bill_tasks['completed'][self.bill_task_ind[0]][self.bill_task_ind[1]] = True
    
    def getRobotTarget(self):
        return self.robot_tasks['position'][self.robot_task_ind[0]][self.robot_task_ind[1]]
        
    def getBillTarget(self):
        return self.bill_tasks['position'][self.bill_task_ind[0]][self.bill_task_ind[1]]
    
    def getRobotCurrentTask(self):
        return self.robot_tasks['cube'][self.robot_task_ind[0]], \
            self.robot_tasks['position'][self.robot_task_ind[0]][self.robot_task_ind[1]], \
            self.robot_task_ind[1]                
                
    def getBillCurrentTask(self):
        return self.bill_tasks['cube'][self.bill_task_ind[0]], \
                self.bill_tasks['position'][self.bill_task_ind[0]][self.bill_task_ind[1]], \
                self.bill_task_ind[1]  
    
    def updateRobotTask(self):
        if self.robot_tasks['completed'][self.robot_task_ind[0]][self.robot_task_ind[1]] and \
            not self.robotFinish :
            if self.robot_task_ind[1] == 0:
                self.robot_task_ind[1] = 1
                return 1 #il lavoro può procedere
            else:
                taskCompleted = self.robot_tasks['cube'][self.robot_task_ind[0]]
                index_taskCompleted = self.project['cube'].index(taskCompleted)
                self.project['completed'][index_taskCompleted] = True
                if self.robot_task_ind[0]+1 == self.max_task:
                    self.robotFinish = True
                    return 2 #tutte le task sono state completate
                self.robot_task_ind[0] = self.robot_task_ind[0] + 1
                self.robot_task_ind[1] = 0
                nextTask = self.robot_tasks['cube'][self.robot_task_ind[0]]
                index_nextTask = self.project['cube'].index(nextTask)
                priorities = self.project['priorities'][index_nextTask]
                if len(priorities) == 0:
                    return 1
                else:
                    priorityBoolList = []
                    for task in priorities:
                        task_index = self.project['cube'].index(task)
                        priorityBoolList.append(self.project['completed'][task_index])
                    if np.array(priorityBoolList).all():
                        return 1
                    else:
                        return 0
        elif self.robotFinish:
            return 2
        else:
            nextTask = self.robot_tasks['cube'][self.robot_task_ind[0]]
            index_nextTask = self.project['cube'].index(nextTask)
            priorities = self.project['priorities'][index_nextTask]
            if len(priorities) == 0:
                return 1
            else:
                priorityBoolList = []
                for task in priorities:
                    task_index = self.project['cube'].index(task)
                    priorityBoolList.append(self.project['completed'][task_index])
                if np.array(priorityBoolList).all():
                    return 1
                else:
                    return 0
    
    def updateBillTask(self):
        if self.bill_tasks['completed'][self.bill_task_ind[0]][self.bill_task_ind[1]] and \
            not self.billFinish :
            if self.bill_task_ind[1] == 0:
                self.bill_task_ind[1] = 1
                return 1 #il lavoro può procedere
            else:
                taskCompleted = self.bill_tasks['cube'][self.bill_task_ind[0]]
                index_taskCompleted = self.project['cube'].index(taskCompleted)
                self.project['completed'][index_taskCompleted] = True
                if self.bill_task_ind[0]+1 == self.max_task:
                    self.billFinish = True
                    return 2 #tutte le task sono state completate
                self.bill_task_ind[0] += 1
                self.bill_task_ind[1] = 0
                nextTask = self.bill_tasks['cube'][self.bill_task_ind[0]]
                index_nextTask = self.project['cube'].index(nextTask)
                priorities = self.project['priorities'][index_nextTask]
                #print(f'Priorities of {nextTask}: {priorities}')
                if len(priorities) == 0:
                    return 1
                else:
                    priorityBoolList = []
                    for task in priorities:
                        task_index = self.project['cube'].index(task)
                        priorityBoolList.append(self.project['completed'][task_index])
                    #print(f'Priorities of {nextTask}: {priorityBoolList}')
                    if np.array(priorityBoolList).all():
                        return 1
                    else:
                        #print(f'{nextTask} ciao:(')
                        return 0
        elif self.billFinish:
            return 2
        else:
            nextTask = self.bill_tasks['cube'][self.bill_task_ind[0]]
            index_nextTask = self.project['cube'].index(nextTask)
            priorities = self.project['priorities'][index_nextTask]
            if len(priorities) == 0:
                return 1
            else:
                priorityBoolList = []
                for task in priorities:
                    task_index = self.project['cube'].index(task)
                    priorityBoolList.append(self.project['completed'][task_index])
                #print(f'Priorities of {nextTask}: {priorityBoolList}')
                if np.array(priorityBoolList).all():
                    #print(f'{nextTask} ciao:)')
                    return 1
                else:
                    #print(f'{nextTask} ciao:(')
                    return 0  
            
            
    def sortTask(self, task, prec):
        numberPrec = [ len(x) for x in prec ]
        numberPrecSort = numberPrec.copy()
        numberPrecSort.sort()
        
        index_task = []
        old_val = None
        eqVal_n = 0
        for val in numberPrecSort:
            if val == old_val:
                eqVal_n += 1
            else:
                eqVal_n = 0 
            indices = [i for i, x in enumerate(numberPrec) if x == val]
            index_task.append(indices[eqVal_n])
            old_val = val
        print(f'index task = {index_task}')
        sortedTask = { 'cube' : [ task['cube'][i] for i in index_task ],
                     'position' : [ task['position'][i] for i in index_task ],
                     'completed' : [ task['completed'][i] for i in index_task ] }
        return sortedTask
        
    def setUpTask(self):
        if self.project is None:
            return
        billLimb = np.array(self.scController.billLimb_spatialPos).copy()
        RightHand_pos = billLimb[6]
        LeftHand_pos = billLimb[7]
        cubes_pos = []
        cubes_pos.append(np.array(self.scController.redCube_pos).copy())
        cubes_pos.append(np.array(self.scController.greenCube_pos).copy())
        cubes_pos.append(np.array(self.scController.blueCube_pos).copy())
        cubes_pos.append(np.array(self.scController.yellowCube_pos).copy())
        cubes_pos.append(np.array(self.scController.greyCube_pos).copy())
        cubes_pos.append(np.array(self.scController.target_pos).copy())
        print(cubes_pos)
        bill_d = []
        for i in range(len(cubes_pos)):
            RX_d = np.linalg.norm([ a-b for a,b in zip(cubes_pos[i],RightHand_pos) ])
            LX_d = np.linalg.norm([ a-b for a,b in zip(cubes_pos[i],LeftHand_pos) ])
            bill_d.append(min(RX_d, LX_d))
        sort_bill_d = bill_d.copy()
        sort_bill_d.sort()
        #print(f'd = {bill_d}', f'sort_d = {sort_bill_d}', sep='\n')
        """
        Trovo gli indici degli oggetti che devono essere prelevati da Bill e
        dal robot sulla base delle distanze (da Bill, il robot va di conseguenza)
        """
        sort_bill_d = sort_bill_d[0:3]
        #print(f'new_sort_d = {sort_bill_d}')
        index_bill_task = []
        """
        for val in sort_bill_d:
            ind = bill_d.index(val)
            index_bill_task.append(ind)
        """
        old_val = None
        eqVal_n = 0
        for val in sort_bill_d:
            if val == old_val:
                eqVal_n += 1
            else:
                eqVal_n = 0 
            indices = [i for i, x in enumerate(bill_d) if x == val]
            index_bill_task.append(indices[eqVal_n])
            old_val = val
        #index_bill_task = [ np.where(bill_d == sort_bill_d[i]) for i in range(3) ]
        index_robot_task = [ ind_r for ind_r in range(6) if ind_r not in index_bill_task ]
        #print(f'Bill index: {index_bill_task}', f'ur10e index: {index_robot_task}', sep='\n')
        
        """
        Si creano le task per robot e operatore
        """
        bill_task = { 'cube' : [ self.project['cube'][i] for i in index_bill_task ],
                      'position' : [ [cubes_pos[i], self.project['position'][i]] for i in index_bill_task ],
                      'completed' : [ [False, False] for _ in range(3) ] }
        robot_task = { 'cube' : [ self.project['cube'][i] for i in index_robot_task ],
                       'position' : [ [cubes_pos[i], self.project['position'][i]] for i in index_robot_task ],
                       'completed' : [ [False, False] for _ in range(3) ] }
        bill_task_priorities = [ self.project['priorities'][i] 
                                for i in index_bill_task ]
        robot_task_priorities = [ self.project['priorities'][i] 
                                for i in index_robot_task ]
        
        #print('old Bill task: {}'.format(bill_task['cube']), 'old ur10e task: {}'.format(robot_task['cube']), sep='\n')
        #print(f'priority bill old task: {bill_task_priorities}')
        
        
        """
        Si ordinano gli oggetti sulla base delle precedenze
        """
        self.bill_tasks = self.sortTask(bill_task, bill_task_priorities)
        self.robot_tasks = self.sortTask(robot_task, robot_task_priorities)
        self.bill_task_ind = [0, 0]
        self.robot_task_ind = [0, 0]
        
        print('Bill task: {}'.format(self.bill_tasks['cube']), 
              'ur10e task: {}'.format(self.robot_tasks['cube']), sep='\n')

    
    def reset(self):
        self.robot_tasks = None
        self.bill_tasks = None
        self.robot_task_ind = None
        self.bill_task_ind = None
        self.robotFinish = False
        self.billFinish = False
        self.project['completed'] = [ False for _ in range(6) ]


class AI:    
    def __init__(self, fc1_searcher=600, fc2_searcher=400, fc3_searcher=200,
                 modelPath_searcher='tmp/ddpg_simplerSearcher_newEnv', inputDim_searcher=(12,),
                 fc1_avoider=900, fc2_avoider=600, fc3_avoider=300,
                 modelPath_avoider='tmp/ddpg_billAvoider', inputDim_avoider=(33,),
                 project=None):
        
        """
        State:
            0 = wait
            1 = work
            2 = all tasks completed        
        """
        
        #Matrice e vettore di rotazione per il cambio di sistema di riferimento di Bill
        self.R_mat = np.array([[0, 1, 0],
                               [-1, 0, 0],
                               [0, 0, 1]])
        self.tr_vect = np.array([0.75, 0.0, 1.0])
        
        self.t_vect_EE = [-0.6, 0.0, 0.0]
        self.t_vect_optitrack = []
        
        self.secure_d = 0.50
        self.warning_d = 0.25
        self.red_fact = 0.5 #fattore di riduzione delle velocità effettive applicate
        self.dt = 0.05
        
        #Velocity Filter Parameters
        self.useFilter = False
        self.action_cntr = 0
        self.speedVect_size = 5
        self.speed_vect = [ [ 0.0 for _ in range(self.vect_size) ] 
                               for c in range(3) ]
        
        self.start_robot_conf = [0.0, -1.57, 1.57, -1.57, 4.71, 2.36]
        
        self.searcher = Agent(input_dims=inputDim_searcher, n_actions=3, fc1=fc1_searcher,
                              fc2=fc2_searcher, fc3=fc3_searcher,
                              chkpt_dir=modelPath_searcher, memory_dir='tmp/memory_simplerSearcher')
        self.avoider = Agent(input_dims=inputDim_avoider, n_actions=3, fc1=fc1_avoider,
                              fc2=fc2_avoider, fc3=fc3_avoider,
                              chkpt_dir=modelPath_avoider, memory_dir='tmp/memory_billAvoider')
        self.controller = Controller()
        self.rate = self.controller.rate
        
        #Target Object
        self.target_bill = np.zeros(3)
        self.target_robot = np.zeros(3)
        
        #Scheduler
        self.scheduler = Scheduler(project, self.controller)
        self.robot_state = -1
        self.bill_state = -1
        
        self.load_weights()
        
        self.resetEnv()
        
    
    def speed_filter(self, action):
        index = self.action_cntr % self.speedVect_size
        for i in range(3):
            self.speed_vect[i][index] = action[i]
        self.action_cntr += 1
        return [ np.mean(self.speed_vect[i]) for i in range(3) ]
    
    def reset_speed_filter(self):
        self.action_cntr = 0
        self.speed_vect = [ [ 0.0 for _ in range(self.vect_size) ] 
                               for c in range(3) ]        
        
    def rate_sleep(self):
        self.rate.sleep()          
    
    def observe_searcher(self, target_pos=None):
        EE_pos, EEvel = self.controller.EE_pos.copy(), self.controller.EE_vel.copy()
        #target_pos = np.array(self.controller.target_pos).copy()
        if target_pos is None:
            target_pos = self.target_robot
        obs = []   
    
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
       
        return obs
    
    
    def observe_avoider(self, v_searcher, theta_target=None):
        theta_joints = np.array(self.controller.robot_th).copy()
        if theta_target is None:
            theta_target = np.zeros(3)
            for i in range(3):
                theta_target[i] = theta_joints[i] + v_searcher[i]*self.dt#*2
        EE_pos = self.controller.EE_pos.copy()
        EE_vel = self.controller.EE_vel.copy()
        billLimbs = [ np.array(self.controller.opHead_pos).copy(), 
                      np.array(self.controller.opRightHand_pos).copy(),
                      np.array(self.controller.opLeftHand_pos).copy() ]
        billLimbs_vel = [ np.array(self.controller.opHead_vel).copy(), 
                          np.array(self.controller.opRightHand_vel).copy(),
                          np.array(self.controller.opLeftHand_vel).copy() ]
        
        obs = []
        
        for l in range(3):
            for k in range(3):
                obs.append(billLimbs[l][k])
            for h in range(3):
                obs.append(billLimbs_vel[l][h])                
        for i in range(3): 
            obs.append(theta_target[i])            
        for i in range(3):   
            obs.append(theta_target[i]-theta_joints[i])          
        for i in range(3):
            obs.append(EE_pos[i])               
        for i in range(3):
            obs.append(EE_vel[i])       
        for i in range(3):
            obs.append(theta_joints[i])

        return obs
    
    
    def checkRobotTarget(self):        
        delta = 0.1 
        d_goal = 0.15
        
        alpha, beta = self.controller.EE_orientation.copy()[0], \
                        self.controlle.EE_orientation.copy()[1]
        alpha_target, beta_target = -1.56, 0.0
        
        EEpos = self.controlle.EE_pos.copy() #se possibile, si effettua sempre la copia dei valori della classe controller
        objPos = np.array(self.controlle.target_pos).copy()
        
        d = 0
        for i in range(np.size(objPos)):
            d = d + (objPos[i] - EEpos[i])**2
        d = np.sqrt(d)
    
        check_bools_pos = d <= d_goal and EEpos[2] >= objPos[2]
        check_angles = [ alpha-alpha_target<=delta and alpha-alpha_target>=-delta,
                         beta-beta_target<=delta and beta-beta_target>=-delta]
    
        check_bools = np.append(check_bools_pos, check_angles)
        taskCompleted = np.array(check_bools).all()
        if taskCompleted and self.robot_state != 2:
            self.controller.robot_vel_publish(np.zeros(6))
            #self.rate.sleep()
            cubeName, cubePos, pickOrPlaceBool = self.scheduler_getCurrentTask(0)
            print('ur10e task completed: {}, {}'.format(cubeName, pickOrPlaceBool))
            x = input('Move Object, then press enter')
            while(x != ''):
                x = input('Move Object, then press enter')
            self.moveObject(cubeName, cubePos, pickOrPlaceBool)
            self.scheduler_clearTask(0)
            self.scheduler_updateTask(0)
            self.target_robot = np.zeros(3)
            if self.robot_state == 1:
                self.scheduler_updateTarget(0)
        return taskCompleted

    
    def checkBillTarget(self):
        RX_hand = np.array(self.controller.opRightHand_pos).copy()
        LX_hand = np.array(self.controller.opLeftHand_pos).copy()
        d_min = 0.1
        rx_d = np.linalg.norm([ RX_hand[i]-self.target_bill[i] for i in range(3) ])
        lx_d = np.linalg.norm([ LX_hand[i]-self.target_bill[i] for i in range(3) ])
        taskCompleted = (rx_d <= d_min) or (lx_d <= d_min)
        if taskCompleted and self.bill_state != 2:
            cubeName, cubePos, pickOrPlaceBool = self.scheduler_getCurrentTask(1)
            print('Bill task completed: {}, {}'.format(cubeName, pickOrPlaceBool))
            self.scheduler_clearTask(1)
            self.scheduler_updateTask(1)
            self.target_bill = np.zeros(3)
            if self.bill_state == 1:
                self.scheduler_updateTarget(1)
        return taskCompleted
    
    def checkAllTarget(self):
        self.checkBillTarget()
        self.checkRobotTarget()
    
    def load_weights(self):
        print('Loading model ...')
        n_steps = 0
        while n_steps <= self.searcher.batch_size:
            obs_sea = self.observe_searcher()
            obs_av = self.observe_avoider(v_searcher=np.zeros(6))
            reward = 0
            done = False
            self.searcher.remember(obs_sea, np.zeros(3), reward, obs_sea, done)
            self.avoider.remember(obs_av, np.zeros(3), reward, obs_av, done)
            n_steps += 1
        self.searcher.learn()
        self.avoider.learn()
        self.searcher.my_load_models(evaluate=True)
        self.avoider.my_load_models(evaluate=True)
        print('Loading completed:)')
        
    def computeSpatialPosJoints(self):
        #TODO: rimuovere i 90deg dal giunto 0 prima di effettuare il calcolo!
        theta = np.array(self.controller.robot_th).copy()
        A_1 = AH(1, theta, 0)
        A_2 = AH(2, theta, 0)
        A_3 = AH(3, theta, 0)
        A_4 = AH(4, theta, 0)
        A_5 = AH(5, theta, 0)
        T = [ A_1,
              A_1*A_2,
              A_1*A_2*A_3,
              A_1*A_2*A_3*A_4,
              A_1*A_2*A_3*A_4*A_5 ]
        spatialPos_joints_robot = [ [ T[i][j,3]-self.t_vect_EE[j] for j in range(3) ]
                                    for i in range(5) ]
        return spatialPos_joints_robot
        
    def find_dMin(self):
        spatialPos_joints_robot = self.computeSpatialPosJoints()
        point_op = [ np.array(self.controller.opHead_pos).copy(), 
                     np.array(self.controller.opRightHand_pos).copy(),
                     np.array(self.controller.opLeftHand_pos).copy() ]
        EE_pos = self.controller.EE_pos.copy()
    
        coll_d = 0.15 #minima distanza (di sicurezza) entro la quale si considera una collisione
        points = []
        points.append(EE_pos)
        for i in range(5, 0, -1):
            points.append(spatialPos_joints_robot[i])
    
        d_min = 100 #distanza minima tra le varie distanze calcolate

        check_d = []
        for k in range(len(points)-1):
            d_limbs =  []
            for l in range(len(point_op)):
                d, _ = pnt2line(point_op[l], points[k], points[k+1])
                check_d.append(d <= coll_d)
                d_limbs.append(d)
                if d<d_min:
                    d_min = d
        collisionBool = np.array(check_d).any()
        if collisionBool:
            print('Collision:(')
        return collisionBool, d_min


    def get_thetaJoints(self):
        return np.array(self.controller.robot_th).copy()
    
    def getRobotState(self):
        return self.robot_state
    
    def getBillState(self):
        return self.bill_state
    
    
    def findV45(self, ctrl):
        #TODO: controllare dir e k!!!
        k = 10
        max_v = 0.5
        alpha, beta = ctrl.EE_orientation.copy()[0], ctrl.EE_orientation.copy()[1]
        alpha_target, beta_target = -1.56, 0.0
        d1r4 = 0 if beta < beta_target else 1
        d1r5 = 1 if beta < beta_target else 0
        mag = np.sqrt((alpha-alpha_target)**2 + (beta-beta_target)**2)
        v4 = max_v * tanh(mag/k) * (-1)**d1r4 # if ... else 0.4
        v5 = max_v * tanh(mag/k) * (-1)**d1r5 # if ... else 0.4
        return v4, v5
    
    def reset_robot_conf(self):
        #TODO: controllare dir
        k = 10
        max_v = 0.5
        theta = np.array(self.controller.robot_th).copy()[0:3]
        theta_target = self.start_robot_conf[0:3]
        v = np.zeros(6)
        for i in range(3):
            d1r = 0 if theta[i]<theta_target[i] else 1
            v[i] = max_v * tanh(abs(theta[i]-theta_target[i])/k) * (-1)**d1r
        v[3], v[4] = self.findV45(self.controller)
        return v        
    
    def doAction(self, target_pos=None, theta_target=None):
        if self.robot_state == 0 or self.robot_state == -1:
            self.scheduler_updateTask(0)
            if self.robot_state == 1:
                self.scheduler_updateTarget(0)
            else:
                #TODO: anziché fare questo si può impostare un target lontano e in alto
                self.reset_speed_filter()
                v = self.reset_robot_conf()
                self.controller.robot_vel_publish(v)
                #self.rate.sleep()
                return
        elif self.robot_state == 2:
            self.reset_speed_filter()
            v = self.reset_robot_conf()
            self.controller.robot_vel_publish(v)
            #self.rate.sleep()
            return
        else:
            self.scheduler_updateTarget(0)
        v4, v5 = self.findV45()
        obs_sea = self.observe_searcher(target_pos)
        v_sea = self.searcher.choose_action(obs_sea, True)
        v_searcher = list(np.array(v_sea).copy())
        v_searcher.append(v4)
        v_searcher.append(v5)
        v_searcher.append(0.0)
        obs_av = self.observe_avoider(v_searcher, theta_target)
        v_av = self.avoider.choose_action(obs_av, True)
        collision, dmin = self.find_dMin()
        v = np.zeros(6)
        for i in range(3):
            if dmin >= self.secure_d:
                v[i] = self.red_fact*v_sea[i]
            elif dmin <= self.warning_d:
                v[i] = self.red_fact*v_av[i]*1.8
            else:
                v[i] = self.red_fact*(dmin*v_sea[i] + (self.secure_d-dmin)*v_av[i])/self.secure_d
        v[3], v[4] = v4, v5
        if self.useFilter:
            v[0:3] = self.speed_filter(v[0:3])
        self.controller.robot_vel_publish(v)
        #self.rate.sleep()
        
        return collision
        
        
    def resetEnv(self): 
        self.controller.robot_vel_publish(np.zeros(6))
        self.scheduler.reset()       
        self.target_bill = np.zeros(3)
        self.target_robot = np.zeros(3)
        self.robot_state = -1
        self.bill_state = -1
        
    def resetBillAndUr10e(self):
        self.controller.billHandPos_publishFun([2,0,0,0])
        self.controller.robot_pos_publish(reset=True)
        
    
    def isProjectCompleted(self):
        return (self.robot_state == 2) and (self.bill_state == 2)
        
        
    #Scheduler Functions
    def scheduler_setUpTask(self):
        self.scheduler.setUpTask()    
    
    def scheduler_updateTask(self, who):
        """
        who:
            0 = robot
            1 = Bill
        """
        if who == 0:
            self.robot_state = self.scheduler.updateRobotTask()
        elif who == 1:
            self.bill_state = self.scheduler.updateBillTask()
        else:
            print('Error in AI class while updating task:(')
    
    def scheduler_updateTarget(self, who):
        """
        who:
            0 = robot
            1 = Bill
        """
        if who == 0:
            self.target_robot = self.scheduler.getRobotTarget()
        elif who == 1:
            self.target_bill = self.scheduler.getBillTarget()
        else:
            print('Error in AI class while updating target:(')
            
    def scheduler_clearTask(self, who):
        """
        who:
            0 = robot
            1 = Bill
        """
        if who == 0:
            self.scheduler.clearRobotTask()
        elif who == 1:
            self.scheduler.clearBillTask()
        else:
            print('Error in AI class while clearing task:(')
            
    def scheduler_getCurrentTask(self, who):
        """
        who:
            0 = robot
            1 = Bill
        """
        if who == 0:
            return self.scheduler.getRobotCurrentTask()
        elif who == 1:
            return self.scheduler.getBillCurrentTask()
        else:
            print('Error in AI class while getting current task:(')
            
            
            