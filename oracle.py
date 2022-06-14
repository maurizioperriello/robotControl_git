#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon May 16 10:01:53 2022

@author: mauri
"""

import os
import numpy as np
import pandas as pd
import tensorflow as tf
from tensorflow import keras
from tensorflow.keras import layers

"""
file = 'tmp/bill.csv'
data = pd.read_csv(file)
data_x = data['position']
data_y = data['target']
"""

print(np.__version__)
print(tf.__version__)

R_mat = np.array([[0, 1, 0],
                  [-1, 0, 0],
                  [0, 0, 1]])
tr_vect = np.array([0.75, 0.0, 1.0])

memory_file = 'tmp/rnn/rnn_weights.h5'

path = 'tmp/bill_move'
elements = os.listdir(path)
file = []
for item in elements:
    file_path = os.path.join(path, item)
    if os.path.isfile(file_path):
        file.append(file_path)
#os.sys.exit()
file = [file[2]]

data_x = []
data_y_nonTrasformati = []
for f in file:
    data = pd.read_csv(f)
    x = data['position']
    for item in x:
        item = item.replace('[[', '')
        item = item.replace(']]', '')
        tmp_l = item.split('], [')
        l = []
        for s in tmp_l:
            s = [ float(number) for number in s.split(', ') ]
            l.append(s)
        data_x.append(l) #size = 8805
    y = data['target']
    for item in y:
        item = item.replace('[', '')
        item = item.replace(']', '')
        l = [ float(n) for n in item.split(', ') ]
        data_y_nonTrasformati.append(l)

#os.sys.exit()

data_y = []
for d in data_y_nonTrasformati:
    d = pd.to_numeric(d)
    d = np.matmul(R_mat, d)
    d = [ d[i]+tr_vect[i] for i in range(3) ]
    data_y.append(d)

print('Data loaded and converted:)')

#os.sys.exit()

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
        seq = np.array(seq).reshape(30*3, -1)
        seq = [ float(seq[i]) for i in range(seq.size) ]
        train_x.append(seq)
        train_y.append(data_y[k])
#print(len(list(data['position'][92])))



index = int(len(train_x)*4/5)
eval_x = []
eval_y = []
for i in range(len(train_x)-1,index,-1):
    eval_x.append(train_x.pop(i))
    eval_y.append(train_y.pop(i))
    
#import os; os.sys.exit()
#os.sys.exit()
#input_size = 3
#sequence_length = 30

print('start RNN:)')

#RNN
model = keras.models.Sequential()
model.add(keras.Input(shape=(30*3,)))
#model.add(layers.SimpleRNN(128, return_sequences=True))
#model.add(layers.LSTM(128))
#model.add(layers.SimpleRNN(128))
model.add(layers.Dense(128))
model.add(layers.Dense(64))
model.add(layers.Dense(32))
model.add(layers.Dense(1))

print(model.summary())

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
