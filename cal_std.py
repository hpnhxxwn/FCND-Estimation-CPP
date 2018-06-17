
# coding: utf-8

# In[8]:

import numpy as np
import pandas as pd


# In[11]:

log1 = np.loadtxt('Graph1.txt', delimiter=',', skiprows=1)


# In[16]:

np.std(log1[:, 1])


# In[17]:

log2 = np.loadtxt('Graph2.txt', delimiter=',', skiprows=1)


# In[18]:

np.std(log2[:, 1])


# In[ ]:



