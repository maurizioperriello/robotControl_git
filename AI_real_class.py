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
from utils import pnt2line, AH, compute_ur_jacobian
from math import pi, copysign
from scipy.spatial.transform import Rotation as R


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
        self.max_task = 2
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
        #billLimb = np.array(self.scController.billLimb_spatialPos).copy()
        RightHand_pos = np.array(self.scController.opRightHand_pos).copy()
        LeftHand_pos = np.array(self.scController.opRightHand_pos).copy()
        cubes_pos = []
        cubes_pos.append(np.array(self.scController.redCube_pos).copy())
        cubes_pos.append(np.array(self.scController.greenCube_pos).copy())
        cubes_pos.append(np.array(self.scController.blueCube_pos).copy())
        cubes_pos.append(np.array(self.scController.yellowCube_pos).copy())
        #cubes_pos.append(np.array(self.scController.greyCube_pos).copy())
        #cubes_pos.append(np.array(self.scController.target_pos).copy())
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
        sort_bill_d = sort_bill_d[0:2]
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
        index_robot_task = [ ind_r for ind_r in range(4) if ind_r not in index_bill_task ]
        #print(f'Bill index: {index_bill_task}', f'ur10e index: {index_robot_task}', sep='\n')
        
        """
        Si creano le task per robot e operatore
        """
        bill_task = { 'cube' : [ self.project['cube'][i] for i in index_bill_task ],
                      'position' : [ [cubes_pos[i], self.project['position'][i]] for i in index_bill_task ],
                      'completed' : [ [False, False] for _ in range(2) ] }
        robot_task = { 'cube' : [ self.project['cube'][i] for i in index_robot_task ],
                       'position' : [ [cubes_pos[i], self.project['position'][i]] for i in index_robot_task ],
                       'completed' : [ [False, False] for _ in range(2) ] }
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
        print(f'bill p = {bill_task_priorities}',
              f'robot p = {robot_task_priorities}', sep='\n')

    
    def reset(self):
        self.robot_tasks = None
        self.bill_tasks = None
        self.robot_task_ind = None
        self.bill_task_ind = None
        self.robotFinish = False
        self.billFinish = False
        self.project['completed'] = [ False for _ in range(4) ]


class AI:    
    def __init__(self, fc1_searcher=600, fc2_searcher=400, fc3_searcher=200,
                 modelPath_searcher='goodW/searcher', inputDim_searcher=(12,),
                 fc1_avoider=900, fc2_avoider=600, fc3_avoider=300,
                 modelPath_avoider='goodW/avoider', inputDim_avoider=(33,),
                 project=None):
        
        """
        State:
            0 = wait
            1 = work
            2 = all tasks completed        
        """
        
        self.t_vect_EE = [-0.6, 0.0, 0.0]
        self.R_mat_ur10e = np.array([[0, -1, 0],
                                     [1, 0, 0],
                                     [0, 0, 1]])
        self.th90rad = -90*2*pi/360
        
        self.secure_d = 0.50
        self.warning_d = 0.25
        self.red_fact = 0.1 #fattore di riduzione delle velocità effettive applicate
        self.dt = 0.05
        
        self.reduction_factor = 0.5 #fattore di riduzione da passare ai costruttori degli agenti
        
        #Velocity Filter Parameters
        self.useFilter = False
        self.action_cntr = 0
        self.filteredAction = 6
        self.speedVect_size = 20
        self.speed_vect = [ [ 0.0 for _ in range(self.speedVect_size) ] 
                               for c in range(self.filteredAction) ]
        
        self.start_robot_conf = [0.0, -1.57, 1.57, -1.57, 4.71, 2.36]
        
        self.searcher = Agent(input_dims=inputDim_searcher, n_actions=3, fc1=fc1_searcher,
                              fc2=fc2_searcher, fc3=fc3_searcher,
                              chkpt_dir=modelPath_searcher, memory_dir='tmp/memory_simplerSearcher',
                              reduction_factor=self.reduction_factor)
        self.avoider = Agent(input_dims=inputDim_avoider, n_actions=3, fc1=fc1_avoider,
                              fc2=fc2_avoider, fc3=fc3_avoider,
                              chkpt_dir=modelPath_avoider, memory_dir='tmp/memory_billAvoider',
                              reduction_factor=self.reduction_factor)
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
        for i in range(self.filteredAction):
            self.speed_vect[i][index] = action[i]
        self.action_cntr += 1
        return [ np.mean(self.speed_vect[i])
                 for i in range(self.filteredAction) ]
    
    def reset_speed_filter(self):
        self.action_cntr = 0
        self.speed_vect = [ [ 0.0 for _ in range(self.speedVect_size) ] 
                               for c in range(self.filteredAction) ]        
        
    def rate_sleep(self):
        self.rate.sleep()          
    
    def observe_searcher(self, action, target_pos=None):
        EE_pos = self.controller.tf_translation.copy()
        """
        #Per passare dalla posizione del TCP a quella dell'EE,
        #come in simulazione
        T_d = np.matrix(np.identity(4), copy=False)
        T_d[2,3] = 0.195
        EE_pos.append(1.0)
        EE_pos = np.dot(T_d, EE_pos)
        EE_pos = [ EE_pos[0,i] for i in range(3) ]
        """
        th = self.controller.robot_th.copy()
        th[0] = th[0] - self.th90rad
        J = compute_ur_jacobian(th)
        EEvel = np.dot(J, action)[0:3]
        EEvel = np.matmul(self.R_mat_ur10e, EEvel)
        
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
    
    
    def observe_avoider(self, v_searcher, action, theta_target=None):
        theta_joints = np.array(self.controller.robot_th).copy()
        if theta_target is None:
            theta_target = np.zeros(3)
            for i in range(3):
                theta_target[i] = theta_joints[i] + v_searcher[i]*self.dt#*2
        EE_pos = self.controller.tf_translation.copy()
        """
        #Per passare dalla posizione del TCP a quella dell'EE,
        #come in simulazione
        T_d = np.matrix(np.identity(4), copy=False)
        T_d[2,3] = 0.195
        EE_pos.append(1.0)
        EE_pos = np.dot(T_d, EE_pos)
        EE_pos = [ EE_pos[0,i] for i in range(3) ]
        """
        th = self.controller.robot_th.copy()
        th[0] = th[0] - self.th90rad
        J = compute_ur_jacobian(th)
        EE_vel = np.dot(J, action)[0:3]
        EE_vel = np.matmul(self.R_mat_ur10e, EE_vel)
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
                        self.controller.EE_orientation.copy()[1]
        alpha_target, beta_target = -1.56, 0.0
        
        EEpos = self.controller.tf_translation.copy() #se possibile, si effettua sempre la copia dei valori della classe controller
        objPos = np.array(self.target_robot).copy()
        
        d = 0
        for i in range(np.size(objPos)):
            d = d + (objPos[i] - EEpos[i])**2
        d = np.sqrt(d)
    
        check_bools_pos = d <= d_goal and EEpos[2] >= objPos[2]
        check_angles = [ alpha-alpha_target<=delta and alpha-alpha_target>=-delta,
                         beta-beta_target<=delta and beta-beta_target>=-delta]
    
        check_bools = np.append(check_bools_pos, check_angles)
        taskCompleted = np.array(check_bools_pos).all()
        if taskCompleted and self.robot_state != 2:
            #self.controller.robot_vel_publish(np.zeros(6))
            #self.rate.sleep()
            cubeName, cubePos, pickOrPlaceBool = self.scheduler_getCurrentTask(0)
            print('ur10e task completed: {}, {}'.format(cubeName, pickOrPlaceBool))
            #self.controller.ctrlSleep(5)
            """
            x = input('Move Object, then press enter')
            while(x != ''):
                x = input('Move Object, then press enter')
            """
            self.scheduler_clearTask(0)
            self.scheduler_updateTask(0)
            self.target_robot = np.zeros(3)
            if self.robot_state == 1:
                self.scheduler_updateTarget(0)
        return taskCompleted

    
    def checkBillTarget(self):
        RX_hand = np.array(self.controller.opRightHand_pos).copy()
        LX_hand = np.array(self.controller.opLeftHand_pos).copy()
        d_min = 0.25
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
            obs_sea = np.zeros(12)
            obs_av = np.zeros(33)
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
        theta = np.array(self.controller.robot_th).copy()    
        theta[0] = theta[0] - self.th90rad
        th = np.matrix([ [theta[i]] for i in range(6) ])
        A_1 = AH(1, th, 0)
        A_2 = AH(2, th, 0)
        A_3 = AH(3, th, 0)
        A_4 = AH(4, th, 0)
        A_5 = AH(5, th, 0)
        T_vect = [ A_1,
                   A_1*A_2,
                   A_1*A_2*A_3,
                   A_1*A_2*A_3*A_4,
                   A_1*A_2*A_3*A_4*A_5 ]
        spatialPos_joints_robot = []
        for i in range(5):
            T = T_vect[i]
            pos_tmp = np.matmul(self.R_mat_ur10e, [ T[i,3] for i in range(3) ])
            spatialPos_joints_robot.append([ pos_tmp[j]+self.t_vect_EE[j]
                                            for j in range(3) ])
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
        for i in range(4, 0, -1):
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
    
    
    def findV456(self):
        k = 500
        max_v = 0.2
        orientation_quat_des = self.controller.desired_quat
        
        th90rad = -90*2*pi/360
        th = self.controller.robot_th.copy()
        th[0] = th[0] - th90rad
        
        orientation_quat = self.controller.tf_orientation.copy()
        if np.dot(orientation_quat_des, orientation_quat) < 0.0:
            orientation_quat = - orientation_quat
        err_quat = R.from_matrix( R.from_quat(orientation_quat).inv().as_matrix() * \
                                  R.from_quat(orientation_quat_des).as_matrix() ).as_quat()
        err = np.dot(-R.from_quat(orientation_quat).as_matrix(), err_quat[0:3])
    
        sign = lambda x: copysign(1, x)
        v_alpha = - k * err[0]
        v_beta = - k * err[1]
        v_gamma = - k * err[2]
    
        v_alpha = v_alpha if abs(v_alpha)<max_v else max_v*sign(v_alpha)
        v_beta = v_beta if abs(v_beta)<max_v else max_v*sign(v_beta)
        v_gamma = v_gamma if abs(v_gamma)<max_v else max_v*sign(v_gamma)
        
        v_rot = [v_alpha, v_beta, v_gamma]
        J_inv = np.linalg.inv(compute_ur_jacobian(th))
        J_inv = J_inv[3:6,3:6]
        v4, v5, v6 = np.matmul(J_inv, v_rot)
        
        return v4, v5, v6
    
    def reset_robot_conf(self):
        #TODO: controllare dir
        start_robot_conf = [0.0, -1.57, 1.57, -1.57, 4.71, 2.36]
        k = 1
        max_v = 0.05
        theta = np.array(self.controller.robot_th).copy()
        theta_target = start_robot_conf[0:3]
        theta[0] = theta[0] - self.th90rad
        #print(theta)
        #d1r = [  ]
        v = np.zeros(6)
        sign = lambda x: copysign(1, x)
        for i in range(3):
            #d1r = 0 if theta[i]<theta_target[i] else 1
            v[i] = - k * (theta[i]-theta_target[i]) #* (-1)**d1r
            v[i] = v[i] if abs(v[i])<max_v else max_v*sign(v[i])
            
        v[0] = 0
        v[3], v[4], v[5] = self.findV456()
        return v       
    
    def doAction(self, action, target_pos=None, theta_target=None):
        if self.robot_state == 0 or self.robot_state == -1:
            self.scheduler_updateTask(0)
            if self.robot_state == 1:
                self.scheduler_updateTarget(0)
            else:
                self.reset_speed_filter()
                v = self.reset_robot_conf()
                #self.controller.robot_vel_publish(v)
                #self.rate.sleep()
                return v, False
        elif self.robot_state == 2:
            self.reset_speed_filter()
            v = self.reset_robot_conf()
            #self.controller.robot_vel_publish(v)
            #self.rate.sleep()
            return v, False
        else:
            self.scheduler_updateTarget(0)
        v4, v5, v6 = self.findV456()
        obs_sea = self.observe_searcher(action, target_pos)
        v_sea = self.searcher.choose_action(obs_sea, True)
        v_searcher = list(np.array(v_sea).copy())
        v_searcher.append(v4)
        v_searcher.append(v5)
        v_searcher.append(0.0)
        obs_av = self.observe_avoider(v_searcher, action, theta_target)
        v_av = self.avoider.choose_action(obs_av, True)
        collision, dmin = self.find_dMin()
        v = np.zeros(6)
        for i in range(3):
            if dmin >= self.secure_d:
                v[i] = self.red_fact*v_sea[i]
            elif dmin <= self.warning_d:
                v[i] = self.red_fact*v_av[i]
            else:
                v[i] = self.red_fact*(dmin*v_sea[i] + (self.secure_d-dmin)*v_av[i])/self.secure_d
        v[3], v[4], v[5] = v4, v5, v6
        """
        if self.useFilter:
            v[0:3] = self.speed_filter(v[0:3])
        """
        #self.controller.robot_vel_publish(v)
        #self.rate.sleep()
        
        return v, collision
        
        
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
            
            
            