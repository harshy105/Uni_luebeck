# -*- coding: utf-8 -*-

"""
Created on Tue Dec 15 23:07:49 2020

@author: harsh
"""

import numpy as np
from functions import dist_calculation
from functions import shortest_path

S = np.array([[0,0,0,-1,0,0],[0,0,-1,0,0,0],[0,0,-1,0,-1,0],[0,0,-1,0,-1,0],[0,0,0,0,-1,0]])
D = np.full(S.shape,np.inf)
D[0,0] = 0
m = S.shape[0]
n = S.shape[1]

D = dist_calculation(0,0,S,D,m,n)
print("Lowest Number of Steps for each Node\n")
print(D,"\n")

i_end = m-1
j_end = n-1
short_path = shortest_path(i_end,j_end,D,m,n)
print("The Shortest Path is \n")
print(short_path,"\n")