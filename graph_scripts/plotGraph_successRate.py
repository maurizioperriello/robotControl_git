#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Mar 17 13:23:04 2022

@author: mauri
"""

#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Mar 14 16:08:39 2022

@author: mauri
"""

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

"""
file = 'tmp/score_dataFrame.csv'
data = pd.read_csv(file)

a = []
c = []

for i in range(len(data)):
    a.append(data.iat[i, 1])
    s = data.iat[i, 2]
    s = s.replace(', shape=(), dtype=float32)', '')
    s = s.replace('tf.Tensor(', '')
    c.append(float(s))
    
df = pd.DataFrame(list(zip(a, c)), 
                  columns=['success', 'score'])
x = [ i for i in range(len(df)) ]
"""

file0 = 'tmp/score/score_dataFrame_searcher_newEnv_0.csv' 
file = 'tmp/score/score_dataFrame_searcher_newEnv.csv'
data = pd.read_csv(file)
data0 = pd.read_csv(file0)

successData = []
for i in range(len(data0)):
    successData.append(data0['success'][i])
for i in range(len(data)):
    successData.append(data['success'][i])

#c = c[16:]
dx = 100

meanScore_vect = []
high_values = []
low_values = []
success_rate = []
for i in range(int(len(successData)/dx-1)):
    sample_success = successData[i*dx:(i+1)*dx]
    success_rate.append(sum(sample_success)/dx)
x1 = [ i for i in range(len(success_rate)) ]

"""
#fig = go.Figure()

fig = px.line(df, x=x1, y=meanScore_vect)
fig.add_scatter(df, y='score')
fig.update_yaxes(range = [-100, 0])
fig.show()
"""

plt.plot(x1, success_rate, color='red')

plt.show()

