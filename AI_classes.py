#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Apr 15 11:33:40 2022

@author: mauri
"""

import numpy as np
from random import sample
from ddpg_classes import Agent
from listener_classes import Controller
from utils import pnt2line


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
        
        self.secure_d = 0.50
        self.warning_d = 0.25
        self.red_fact = 0.5 #fattore di riduzione delle velocità effettive applicate
        self.dt = 0.05
        
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
        
        #Object position
        self.xPos = [ 0.25+0.15*i for i in range(4) ]
        self.yPos = [-0.45+0.15*i for i in range(7) ]
        self.zPos = 0.07
        
        self.load_weights()
        
        self.resetEnv()
          
    
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
        theta_joints = np.array(self.controller.robot_pos).copy()
        if theta_target is None:
            theta_target = np.zeros(3)
            #vel_joints = np.array(self.controller.robot_vel).copy()
            for i in range(3):
                theta_target[i] = theta_joints[i] + v_searcher[i]*self.dt*2
        EE_pos = self.controller.EE_pos.copy()
        EE_vel = self.controller.EE_vel.copy()
        billLimbs = np.array(self.controller.billLimb_spatialPos).copy()
        billLimbs_vel = np.array(self.controller.billLimb_spatialVel).copy()
        limbSelector = [4, 6, 7] #testa, mano destra, mano sinistra
        
        obs = []
        
        for l in limbSelector:
            for k in range(3):
                obs.append(billLimbs[l][k])
            for h in range(3):
                obs.append(billLimbs_vel[l][h])                
        for i in range(3):
            #obs.append(object_pos[i]) 
            obs.append(theta_target[i])            
        for i in range(3):
            #obs.append(object_pos[i]-EE_pos[i])   
            obs.append(theta_target[i]-theta_joints[i])          
        for i in range(3):
            obs.append(EE_pos[i])               
        for i in range(3):
            obs.append(EE_vel[i])       
        for i in range(3):
            obs.append(theta_joints[i])

        return obs
    
    
    def checkRobotTarget(self):
        #funzione per controllare se il risultato è stato raggiunto o meno:
        #per farlo, bisogna arrivare in prossimità del target (sfera bianca) con
        #una velocità inferiore o ugule a 0.1 m/s
        delta = 0.02 
        d_goal = 0.15
        
        EEpos = self.controller.EE_pos.copy() #se possibile, si effettua sempre la copia dei valori della classe controller
        objPos = np.array(self.target_robot).copy()
        finalLinks = self.controller.finalLinks_spatialPos.copy()
        joints_spatialPos = self.controller.joints_spatialPos
        
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
        taskCompleted = np.array(check_bools).all()
        if taskCompleted and self.robot_state != 2:
            cubeName, cubePos, pickOrPlaceBool = self.scheduler_getCurrentTask(0)
            print('ur10e task completed: {}, {}'.format(cubeName, pickOrPlaceBool))
            self.moveObject(cubeName, cubePos, pickOrPlaceBool)
            self.scheduler_clearTask(0)
            self.scheduler_updateTask(0)
            self.target_robot = np.zeros(3)
            if self.robot_state == 1:
                self.scheduler_updateTarget(0)
        return taskCompleted

    
    def checkBillTarget(self):
        point_op = np.array(self.controller.billLimb_spatialPos).copy()
        RX_hand = point_op[6]
        LX_hand = point_op[7]
        d_min = 0.1
        rx_d = np.linalg.norm([ RX_hand[i]-self.target_bill[i] for i in range(3) ])
        lx_d = np.linalg.norm([ LX_hand[i]-self.target_bill[i] for i in range(3) ])
        taskCompleted = (rx_d <= d_min) or (lx_d <= d_min)
        if taskCompleted and self.bill_state != 2:
            cubeName, cubePos, pickOrPlaceBool = self.scheduler_getCurrentTask(1)
            print('Bill task completed: {}, {}'.format(cubeName, pickOrPlaceBool))
            self.moveObject(cubeName, cubePos, pickOrPlaceBool)
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
        
        
    def find_dMin(self):
        #RIDURRE IL NUMERO DI PARAGONI (USARE index=[2,4]?)
        spatialPos_joints_robot = self.controller.joints_spatialPos.copy()
        point_op = np.array(self.controller.billLimb_spatialPos).copy()
        EE_pos = self.controller.EE_pos.copy()
    
        coll_d = 0.15 #minima distanza (di sicurezza) entro la quale si considera una collisione
        limbSelector = [4, 6, 7]
        points = []
        points.append(EE_pos)
        for i in range(5, 0, -1):
            points.append(spatialPos_joints_robot[i])
    
        d_min = 100 #distanza minima tra le varie distanze calcolate

        check_d = []
        d_max = 0
        for k in range(len(points)-1):
            d_limbs =  []
            for l in range(len(point_op)):
                d, _ = pnt2line(point_op[l], points[k], points[k+1])
                check_d.append(d <= coll_d)
                if(d > d_max):
                    d_max = d
                if(l in limbSelector):
                    d_limbs.append(d)
                    if d<d_min:
                        d_min = d
        collisionBool = np.array(check_d).any()
        if collisionBool:
            print('Collision:(')
        return collisionBool, d_min


    def get_thetaJoints(self):
        return np.array(self.controller.theta_joints).copy()
    
    def getRobotState(self):
        return self.robot_state
    
    def getBillState(self):
        return self.bill_state
    
    
    def findV45(self):
        k = 30 #si parte da 10, funziona ma è lento
        finalLinks = self.controller.finalLinks_spatialPos.copy()
        EE_alpha = finalLinks[0][0] - finalLinks[1][0] #differenza tra le coordinate x del terzultimo link e dell'EE
        EE_beta = finalLinks[0][1] - finalLinks[1][1]
        z_check = finalLinks[1][2] < finalLinks[0][2]
        d = np.sqrt(EE_alpha**2 + EE_beta**2)
        d1r4 = 1 if finalLinks[0][0]>finalLinks[1][0] else 0 #perché "dir" è meglio non usarlo
        d1r5 = 0 if finalLinks[0][1]>finalLinks[1][1] else 1
        v4 = k * d * (-1)**d1r4 if z_check else 0.7
        v5 = k * d * (-1)**d1r5 if z_check else 0.7
        
        return v4, v5
    
    
    def doAction(self, target_pos=None, theta_target=None):
        if self.robot_state == 0 or self.robot_state == -1:
            self.scheduler_updateTask(0)
            if self.robot_state == 1:
                self.scheduler_updateTarget(0)
            else:
                self.controller.robot_pos_publish(reset=True)
                return
        elif self.robot_state == 2:
            self.controller.robot_pos_publish(reset=True)
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
        self.controller.robot_vel_publish(v)
        
        return collision
        
    
    def moveBill(self, object_target=None):
        if self.bill_state == 0 or self.bill_state == -1:
            self.scheduler_updateTask(1)
            if self.bill_state == 1:
                self.scheduler_updateTarget(1)
            else:
                self.controller.billHandPos_publishFun([2,0,0,0])
                #print('Bill is returning home:)')
                return
        elif self.bill_state == 2:
            self.controller.billHandPos_publishFun([2,0,0,0])
            #print('Bill is returning home:)')
            return
        else:
            self.scheduler_updateTarget(1)
            #print(self.target_bill)
        if object_target is None:
            object_target = self.target_bill
        object_target = np.matmul(self.R_mat, object_target)
        object_target = [ object_target[i]+self.tr_vect[i] for i in range(3) ]
        bill_cmd = np.zeros(4)
        for i in range(3):
            bill_cmd[i+1] = object_target[i]
        if self.target_bill[1]<0 :
            bill_cmd[0] = 0
        else:
            bill_cmd[0] = 1
        self.controller.billHandPos_publishFun(bill_cmd)
        
    def moveObject(self, object_str, pos, pickOrPlaceBool):
        """
        pickOrPlaceBool:
            False = pick
            True = place
        """
        if not pickOrPlaceBool:
            pos[2] = -1.0
        if  object_str == 'red':
            self.controller.redCube_publish(pos)        
        elif  object_str == 'green':
            self.controller.greenCube_publish(pos)
        elif  object_str == 'blue':
            self.controller.blueCube_publish(pos)
        elif  object_str == 'yellow':
            self.controller.yellowCube_publish(pos)
        elif  object_str == 'grey':
            self.controller.greyCube_publish(pos)
        elif  object_str == 'sphere':
            self.controller.target_pos_publish(pos)
        else:
            print('Wrong cube passed in moveObject fun:(')
        
        
    def resetEnv(self): 
        yPosSampled = sample(self.yPos, 6)
        
        redCube_pos = [ sample(self.xPos, 1)[0], yPosSampled[0], self.zPos]
        greenCube_pos = [ sample(self.xPos, 1)[0], yPosSampled[1], self.zPos]
        blueCube_pos = [ sample(self.xPos, 1)[0], yPosSampled[2], self.zPos]
        yellowCube_pos = [ sample(self.xPos, 1)[0], yPosSampled[3], self.zPos]
        greyCube_pos = [ sample(self.xPos, 1)[0], yPosSampled[4], self.zPos]
        sphere_pos = [ sample(self.xPos, 1)[0], yPosSampled[5], self.zPos]
        
        """
        #due gruppi di oggetti ben definiti
        redCube_pos = [0.3, 0.5, 0.07]
        greenCube_pos = [0.6, 0.5, 0.07]
        blueCube_pos = [0.12, 0.5, 0.07]
        yellowCube_pos = [0.12, -0.3, 0.07]
        greyCube_pos = [0.5, -0.3, 0.07]
        sphere_pos = [0.7, -0.3, 0.07]
        """
        """
        #Questa configurazione ha dato problemi nel calcolo degli indici una volta (dopo alcuni reset automatici), poi non più
        redCube_pos = [0.25, -0.45, 0.07]
        greenCube_pos = [0.4, -0.15, 0.07]
        blueCube_pos = [0.7, 0.15, 0.07]
        yellowCube_pos = [0.55, 0.3, 0.07]
        greyCube_pos = [0.5, 0.0, 0.07]
        sphere_pos = [0.7, -0.3, 0.07]
        """
        self.controller.redCube_publish(redCube_pos)
        self.controller.greenCube_publish(greenCube_pos)
        
        self.controller.billHandPos_publishFun([2,0,0,0])
        self.controller.robot_pos_publish(reset=True)
         
        self.controller.blueCube_publish(blueCube_pos)
        self.controller.yellowCube_publish(yellowCube_pos)
        
        resetCompleted = False
        reset_pos_robot = self.controller.start_pos_robot
        r_pos = np.array(self.controller.robot_pos).copy()
        c = 0
        while(not resetCompleted):
            r_pos = np.array(self.controller.robot_pos).copy()
            resetCompleted = np.array(
                [ r_pos[i]>=reset_pos_robot[i]-0.1 and r_pos[i]<=reset_pos_robot[i]+0.1 for i in range(6) ]
                ).all()
            c += 1
            if(c == 500_000):
                break;
        self.controller.robot_vel_publish(np.zeros(6))
        self.controller.greyCube_publish(greyCube_pos)
        self.controller.target_pos_publish(sphere_pos)
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
            
            
            