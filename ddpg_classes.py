#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Nov  5 17:38:18 2021

@author: mauri
"""

#########################################################################################################
#
#   AGENT CLASS, comprende:
#   - classe dell'ACTOR network
#   - classe della CRITIC network
#   - classe del BUFFER di memoria
#
#   L'agente realizzato comprende una rete ACTOR (per stabilire l'azione da compiere),
#   una rete CRITIC (per stabilire il valore dell'azione compiuta, considerando anche
#   lo stato di partenza), e due reti TARGET, una actor e una critic (per stabilire
#   il valore a cui le reti devono tendere)
#
#   La logica dell'apprendimento e della memoria è SARSD:
#   S = State
#   A = Action
#   R = Reward
#   S'= New State
#   D = Terminal Flag
#
#########################################################################################################

import os
import numpy as np
import csv
import tensorflow as tf
import tensorflow.keras as keras
from tensorflow.keras.models import load_model
from tensorflow.keras.optimizers import Adam
from tensorflow.keras.layers import Dense

#CRITIC NETWORK
class CriticNetwork(keras.Model):
    def __init__(self, fc1_dims=512, fc2_dims=512, fc3_dims=512,
                 name='critic', chkpt_dir='tmp/ddpg', model_dir='tmp/model'):
        super(CriticNetwork, self).__init__() #inizializzazione della rete super
        self.fc1_dims = fc1_dims #dimesione primo dense layer
        self.fc2_dims = fc2_dims #dimesione secondo dense layer
        self.fc3_dims = fc3_dims #dimesione terzo dense layer
        self.model_name = name #nome del modello, utile al salvataggio dei pesi
        self.checkpoint_dir = chkpt_dir #cartella in cui salvare i pesi
        self.model_dir = model_dir #cartella in cui salvare il modello, inutile
        self.checkpoint_file = os.path.join(self.checkpoint_dir,
                                            self.model_name+'_ddpg.h5') #file in cui salvare i pesi
        self.model_file = os.path.join(self.model_dir,
                                       self.model_name+'_model.h5py') #file in cui salvare l'intero modello
        
        #NN
        self.fc1 = Dense(self.fc1_dims, activation='relu') #layer 1
        self.fc2 = Dense(self.fc2_dims, activation='relu') #layer 2
        self.fc3 = Dense(self.fc3_dims, activation='relu') #layer 3
        self.q = Dense(1, activation=None) #output layer, il q_value è unico
        
        
    def call(self, state, action):
        #sovrascrittura della funzione eseguita ad ogni chiamata della rete
        #si fanno passare i valori concatenati di stato e azione attraverso ogni layer
        #fino ad ottenere il q value
        action_value = self.fc1(tf.concat([state, action], axis=1))
        action_value = self.fc2(action_value)
        action_value = self.fc3(action_value)
        q = self.q(action_value)
        return q
    
#ACTOR NETWORK
class ActorNetwork(keras.Model):
    def __init__(self, fc1_dims=512, fc2_dims=512, fc3_dims=512, n_actions=6,
                 name='actor', chkpt_dir='tmp/ddpg', model_dir='tmp/model'):
        super(ActorNetwork, self).__init__()
        self.fc1_dims = fc1_dims
        self.fc2_dims = fc2_dims
        self.fc3_dims = fc3_dims
        self.n_actions = n_actions #dimensione dello spazio delle azioni
        self.model_name = name
        self.checkpoint_dir = chkpt_dir
        self.model_dir = model_dir
        self.checkpoint_file = os.path.join(self.checkpoint_dir,
                                            self.model_name+'_ddpg.h5')
        self.model_file = os.path.join(self.model_dir,
                                       self.model_name+'_model.h5py')
        
        #NN
        self.fc1 = Dense(self.fc1_dims, activation='relu')
        self.fc2 = Dense(self.fc2_dims, activation='relu')
        self.fc3 = Dense(self.fc3_dims, activation='relu')
        self.mu = Dense(n_actions, activation='tanh') #si usa la tangente iperbolica per dare dei limiti ai valori delle azioni
        #tali limiti sono poi modificabili a piacere con opportuni coefficienti
        
    def call(self, state):
        act = self.fc1(state)
        act = self.fc2(act)
        act = self.fc3(act)
        
        #sse si vuole cambiare il limite delle azioni, basta moltiplicare qui per opportuni coefficienti
        act = self.mu(act)
        return act
        

#CLASS ReplayBuffer: sistema di memoria

class ReplayBuffer:
    def __init__(self, max_size, input_shape, n_actions, memory_dir):
        #n_actions = number of component of action (because it's a continuous action space)
        self.mem_size = max_size #dimesione della memoria
        #self.mem_cntr = 0 #counter interno per tenere traccia del numero di elementi effettivamente salvati
        self.mem_cntr = 0
        #inizializzazione dei vettori della memoria
        self.state_memory = np.zeros((self.mem_size, *input_shape)) #vettore degli stati presenti
        self.new_state_memory = np.zeros((self.mem_size, *input_shape)) #vettore degli stati futuri
        self.action_memory = np.zeros((self.mem_size, n_actions)) #vettore delle azioni compiute
        self.reward_memory = np.zeros(self.mem_size) #vettore dei reward ottenuti
        self.terminal_memory = np.zeros(self.mem_size, dtype=np.bool) #vettore dei terminal flag, indicano se il target è stato raggiunto o meno
        
        #file csv in cui salvare la memoria
        #se ne usano 5 diversi per comodità, vista la grande dimensione dei vettori
        self.chkptFile_stateMemory = os.path.join(memory_dir, 'stateMemory.csv')
        self.chkptFile_newStateMemory = os.path.join(memory_dir, 'newStateMemory.csv')
        self.chkptFile_actionMemory = os.path.join(memory_dir, 'actionMemory.csv')
        self.chkptFile_rewardMemory = os.path.join(memory_dir, 'rewardMemory.csv')
        self.chkptFile_terminalMemory = os.path.join(memory_dir, 'terminalMemory.csv')
        #si salva anche l'indice a cui si è arrivati
        self.chkptFile_indexMemory = os.path.join(memory_dir, 'indexMemory.csv')
        
    def store_transition(self, state, action, reward, new_state, done):
        #funzione per salvare i dati ottenuti
        index = self.mem_cntr % self.mem_size #in questo modo, una volta superato il limite, si riparte da 0 sovrascrivendo via via i dati più vecchi
        
        self.state_memory[index] = state
        self.new_state_memory[index] = new_state
        self.action_memory[index] = action
        self.reward_memory[index] = reward
        self.terminal_memory[index] = done
        
        self.mem_cntr += 1
        
    def sample_buffer(self, batch_size):
        #funzione per estrarre un batch casuale di dati dalla memoria disponibile
        max_mem = min(self.mem_cntr, self.mem_size) #we never put to zero again mem_cntr, so we take the minimum between these two numbers
        batch = np.random.choice(max_mem, batch_size, replace=False)
        
        states = self.state_memory[batch]
        actions = self.action_memory[batch]
        rewards = self.reward_memory[batch]
        states_ = self.new_state_memory[batch]
        dones = self.terminal_memory[batch]
        
        return states, actions, rewards, states_, dones
    
    def save_memory(self):
        #funzione per salvare la memoria all'interno degli opportuni file csv
        print('Saving memory: 0%')
        with open(self.chkptFile_stateMemory, "w") as file:
            np.savetxt(file, self.state_memory)
        print('Saving memory: 16%')
        with open(self.chkptFile_actionMemory, "w") as file:    
            np.savetxt(file, self.action_memory)
        print('Saving memory: 32%')
        with open(self.chkptFile_rewardMemory, "w") as file:
            np.savetxt(file, self.reward_memory)
        print('Saving memory: 48%')
        with open(self.chkptFile_newStateMemory, "w") as file:
            np.savetxt(file, self.new_state_memory)
        print('Saving memory: 64%')
        with open(self.chkptFile_terminalMemory, "w") as file:
            np.savetxt(file, self.terminal_memory)
        print('Saving memory: 80%')
        with open(self.chkptFile_indexMemory, "w") as file:
            np.savetxt(file, np.array(self.mem_cntr, ndmin=1))
        print('Saving memory: 100%')
            
        
    def load_memory(self):
        #funzione per caricare la memoria dagli opportuni file csv
        self.state_memory = np.genfromtxt(self.chkptFile_stateMemory)
        self.new_state_memory = np.genfromtxt(self.chkptFile_newStateMemory)
        self.action_memory = np.genfromtxt(self.chkptFile_actionMemory)
        self.reward_memory = np.genfromtxt(self.chkptFile_rewardMemory)
        self.terminal_memory = np.genfromtxt(self.chkptFile_terminalMemory)
        self.mem_cntr = int(np.genfromtxt(self.chkptFile_indexMemory))


#CLASS AGENT
class Agent:
    def __init__(self, input_dims, alpha=0.001, beta=0.002, action_limit=[-1, 1],
                 gamma=0.99, n_actions=6, max_size=1_000_000, tau=0.005,
                 fc1=600, fc2=400, fc3=200, batch_size=64, noise=0.1,
                 chkpt_dir='tmp/ddpg', memory_dir='tmp/memory'):
        self.gamma = gamma #discount factor, peso del reward futuro rispetto a quello attuale
        self.n_actions = n_actions #dimensione dello spazio delle azioni
        self.tau = tau #coefficiente che controlla il soft update delle reti target rispetto alle altre due
        self.memory = ReplayBuffer(max_size, input_dims, n_actions, memory_dir) #memoria dell'agente
        self.batch_size = batch_size #grandezza del batch da estrarre dalla memoria per il training
        self.noise = noise #deviazione standard del rumore gaussiano da aggiungere all'azione per favorire una maggiore esplorazione dell'ambiente
        self.max_action = [action_limit[1] for _ in range(n_actions)] #massima azione consentita
        self.min_action = [action_limit[0] for _ in range(n_actions)] #minima azione consentita

        self.noise_counter = 0 #serve per valutare ogni quanto abbassare il rumore

        #definizione delle 4 reti dell'agente
        self.actor = ActorNetwork(fc1_dims=fc1, fc2_dims=fc2, fc3_dims=fc3,
                                  n_actions=n_actions, name='actor',chkpt_dir=chkpt_dir)
        self.critic = CriticNetwork(fc1_dims=fc1, fc2_dims=fc2, fc3_dims=fc3,
                                    name='critic',chkpt_dir=chkpt_dir)
        self.target_actor = ActorNetwork(fc1_dims=fc1, fc2_dims=fc2, fc3_dims=fc3,
                                         n_actions=n_actions, name='target_actor',chkpt_dir=chkpt_dir)
        self.target_critic = CriticNetwork(fc1_dims=fc1, fc2_dims=fc2, fc3_dims=fc3,
                                           name='target_critic',chkpt_dir=chkpt_dir)
        
        #selezione del compilatore delle reti con ottimizzatore Adam
        self.actor.compile(optimizer=Adam(learning_rate=alpha))
        self.critic.compile(optimizer=Adam(learning_rate=beta))
        self.target_actor.compile(optimizer=Adam(learning_rate=alpha))
        self.target_critic.compile(optimizer=Adam(learning_rate=beta))
        
        self.update_network_parameters(tau=1)
        
    def update_network_parameters(self, tau=None):
        #funzione per aggiornare i parametri delle reti target
        #inizialmente si effettua una hard copy dei parametri delle reti di partenza
        #in quelle target, dopo si opta per un soft update.
        #In questo modo le reti target risentono meno delle varie oscillazioni 
        #dei pesi delle reti non target
        
        #target_weights = nonTarget_weight*tau + target_weights*(1-tau)
        
        if tau is None:
            tau = self.tau
            
        weights = []
        targets = self.target_actor.weights
        for i,weight in enumerate(self.actor.weights):
            weights.append(weight*tau + targets[i]*(1-tau))
        self.target_actor.set_weights(weights)
        
        weights = []
        targets = self.target_critic.weights
        for i,weight in enumerate(self.critic.weights):
            weights.append(weight*tau + targets[i]*(1-tau))
        self.target_critic.set_weights(weights)
        
    def remember(self, state, action, reward, new_state, done):
        #funzione per richiamare la memoria interna (è più corretto fare così,
        #con chiamate indirette delle funzioni delle varie classi che fanno parte
        #dell'agente)
        self.memory.store_transition(state, action, reward, new_state, done)
        
    def save_models(self, count):
        #funzione per salvare i pesi, la memoria e i modelli (non utilizzati)
        print('Saving models ...')
        self.actor.save_weights(self.actor.checkpoint_file)
        self.critic.save_weights(self.critic.checkpoint_file)
        self.target_actor.save_weights(self.target_actor.checkpoint_file)
        self.target_critic.save_weights(self.target_critic.checkpoint_file)
        
        #SALVARE MEMORIA!!!
        if(count % 100 == 0 and count != 0):
            self.memory.save_memory()
        
        """
        self.actor.save(self.actor.model_file, save_format='tf')
        self.critic.save(self.critic.model_file, save_format='tf')
        self.target_actor.save(self.target_actor.model_file, save_format='tf')
        self.target_critic.save(self.target_critic.model_file, save_format='tf')
        
        
        self.actor.save(self.actor.model_file)
        self.critic.save(self.critic.model_file)
        self.target_actor.save(self.target_actor.model_file)
        self.target_critic.save(self.target_critic.model_file)
        """

    def my_load_models(self):
        #funzione per caricare i pesi delle reti e la memoria
        print('Loading models...')
        self.actor.load_weights(self.actor.checkpoint_file)
        self.critic.load_weights(self.critic.checkpoint_file)
        self.target_actor.load_weights(self.target_actor.checkpoint_file)
        self.target_critic.load_weights(self.target_critic.checkpoint_file)
        
        #CARICARE MEMORIA!!!
        print('Loading memory...')
        self.memory.load_memory()
    
    def load_complete_models(self):
        #funzione per caricare i modelli interi della rete (non utilizzata)
        self.actor = load_model(self.actor.model_file)
        self.critic = load_model(self.critic.model_file)
        self.target_actor = load_model(self.target_actor.model_file)
        self.target_critic = load_model(self.target_critic.model_file)

    def choose_action(self, observation, evaluate=False):
        #funzione per determinare l'azione da compiere
        
        self.noise_counter += 1
        if(self.noise_counter == 900_000):
            #ogni 300 episodi si abbassa il rumore (ogni episodio sono 3000 step)
            if(self.noise > 0):
                self.noise -= 0.01
                self.noise_counter = 0
        
        state = tf.convert_to_tensor([observation], dtype=tf.float32) #conversione in tensore del vettore delle osservazioni
        actions = self.actor(state) #definizione dell'azione grazie alla rete actor
        if not evaluate and self.noise!=0 :
            #se non si è nell'evaluation, allora si aggiunge rumore gaussiano
            actions += tf.random.normal(shape=[self.n_actions],
                                        mean=0.0,
                                        stddev=self.noise)
        #si riducono i valori dell'azione ai limiti (nel caso li superino col rumore)
        actions = tf.clip_by_value(actions, self.min_action, self.max_action)
        
        return actions[0] #il primo valore del vettore è quello che interessa (ossia il vettore con le sei velocità  da applicare ai giunti)
    
    def learn(self):
        #print('Learning:)')
        if self.memory.mem_cntr < self.batch_size:
            #se non ci sono abbastanza elementi in memoria, allora non si effettua la procedura di learning
            return
        
        state, action, reward, new_state, done = \
            self.memory.sample_buffer(self.batch_size) #si estrae un batch di dati dalla memoria
          
        #si convertono i vettori in tensori di tf
        states = tf.convert_to_tensor(state, dtype=tf.float32)
        states_ = tf.convert_to_tensor(new_state, dtype=tf.float32)
        actions = tf.convert_to_tensor(action, dtype=tf.float32)
        rewards = tf.convert_to_tensor(reward, dtype=tf.float32) 
        #non è necessario convertire i terminal flags
        
        #critic loss
        with tf.GradientTape() as tape:
            target_actions = self.target_actor(states_) #azione per lo stato futuro
            #valore dell'azione futura e dello stato futuro
            critic_value_ = tf.squeeze(self.target_critic(
                                            states_, target_actions), 1) #squeeze è usato per rimuovere l'indicazione della dimesione del batch_size dal tensore
            #valore dell'azione presente, dato lo stato presente
            critic_value = tf.squeeze(self.critic(
                                            states, actions), 1)
            target = rewards + self.gamma*critic_value_*(1-done) #se l'episodio è completato (done=1), allora il reward è pari al target
            critic_loss = keras.losses.MSE(target, critic_value) #Mean Squared Error tra il target e il valore ottenuto
        
        #calcolo del gradiente della funzione di perdita
        critic_network_gradient = tape.gradient(critic_loss,
                                                self.critic.trainable_variables)
        #applicazione del gradiente e learning
        self.critic.optimizer.apply_gradients(zip(
            critic_network_gradient, self.critic.trainable_variables))
        
        #actor loss
        with tf.GradientTape() as tape:
            new_policy_actions = self.actor(states) #azione calcolata dalla rete actor
            actor_loss = -self.critic(states, new_policy_actions) #si vuole massimizzare il punteggio ottenuto, preciò bisogna seguire la direzione negativa del gradiente
            
            actor_loss = tf.math.reduce_mean(actor_loss) #media degli elementi del vettore
            
        #la loss ha cambiato segno, quindi si può considerare il suo gradiente per individuare la direzione di massima crescita del punteggio
        actor_network_gradient = tape.gradient(actor_loss,
                                               self.actor.trainable_variables)
        self.actor.optimizer.apply_gradients(zip(
            actor_network_gradient, self.actor.trainable_variables))
        
        self.update_network_parameters() #una volta aggiornati i pesi delle reti actor e critic, si aggiornano quelli delle reti target
            
            
            
            
            
            
            
