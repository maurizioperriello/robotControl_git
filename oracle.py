#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon May 16 10:01:53 2022

@author: mauri
"""

import numpy as np
import pandas as pd
import tensorflow as tf
from tensorflow import keras
from tensorflow.keras import layers

memory_file = 'tmp/rnn/rnn_weights.h5'

file = 'tmp/bill.csv'
data = pd.read_csv(file)
data_x = data['position']
data_y = data['target']

#data_xx = [ [ [ j for j in range(3*i,3*(i+1)) ] for i in range(63) ] for _ in range(10)]
#data_yy = [ [ i for i in range(3*j,3*(j+1)) ] for j in range(10) ]

sequence_length = 30
train_x = []
train_y = []
for k in range(len(data_x)):
    d = data_x[k]
    seq_n = int(len(d)/sequence_length)
    delta = len(d) % sequence_length
    for i in range(seq_n,0,-1):
        seq = []
        for j in range(30*(i-1),30*i):
            seq.append(d[j+delta])
        train_x.append(seq)
        train_y.append(data_y[k])
#print(len(list(data['position'][92])))

index = int(len(train_x)*4/5)
eval_x = []
eval_y = []
for i in range(len(train_x)-1,index,-1):
    eval_x.append(train_x.pop(i))
    eval_y.append(train_y.pop(i))
    
import os; os.sys.exit()

#input_size = 3
#sequence_length = 30

#RNN
model = keras.models.Sequential()
model.add(keras.Input(shape=(3,30)))
#model.add(layers.SimpleRNN(128), return_sequences=True)
#model.add(layers.LSTM(128))
model.add(layers.SimpleRNN(128))
model.add(layers.dense(3))

#loss and optimizer
loss = keras.losses.MeanSquaredError()
optim = keras.optimizers.Adam(lr=0.001)
metrics = ['accuracy']

model.compile(loss=loss, optimizer=optim, metrics=metrics)

#training
batch_size = 64
epochs = 10

model.fit(train_x, train_y, batch_size=batch_size, epochs=epochs, verbose=2)

#evaluate
model.evaluate(eval_x, eval_y, batch_size=batch_size, verbose=2)

model.save(memory_file)
