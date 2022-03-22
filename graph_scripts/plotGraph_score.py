#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Mar 20 17:18:21 2022

@author: mauri
"""

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

#file = 'tmp/score_dataFrame.csv'
file0 = 'tmp/score/score_dataFrame_searcher_newEnv_0.csv' 
file = 'tmp/score/score_dataFrame_searcher_newEnv.csv'
data = pd.read_csv(file)
data0 = pd.read_csv(file0)

scoreData = []
for i in range(len(data0)):
    scoreData.append(data0['score'][i])
for i in range(len(data)):
    scoreData.append(data['score'][i])

print('lenght = {}'.format(len(scoreData)))

#c = c[16:]
dx = 500
#print('rnd_value = {}'.format(data['score'][6:9]))
meanScore_vect = []
high_values = []
low_values = []
for i in range(int(len(scoreData)/dx-1)):
    #sample = [ data['score'][k] for k in range(i*dx, (i+1)*dx) ]
    #sample = data['score'][i*dx:(i+1)*dx]
    sample = scoreData[i*dx:(i+1)*dx]
    mean = np.mean(sample)
    up_val = []
    down_val = []
    for j in range(dx):
        if(sample[j] > mean):
            up_val.append(sample[j])
        elif(sample[j] <= mean):
            down_val.append(sample[j])
    high_values.append(np.mean(up_val))
    low_values.append(np.mean(down_val))
    meanScore_vect.append(mean)
x1 = [ i for i in range(len(meanScore_vect)) ]

"""
#fig = go.Figure()

fig = px.line(df, x=x1, y=meanScore_vect)
fig.add_scatter(df, y='score')
fig.update_yaxes(range = [-100, 0])
fig.show()
"""

plt.plot(x1, meanScore_vect, color='blue')
plt.plot(x1, high_values, color='grey')
plt.plot(x1, low_values, color='grey')
plt.fill_between(x1, high_values, meanScore_vect, color='lightgrey')
plt.fill_between(x1, meanScore_vect, low_values, color='lightgrey')

plt.show()

