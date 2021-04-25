# -*- coding: utf-8 -*-
"""
Created on Wed Dec 16 20:06:12 2020

@author: harsh
"""

import numpy as np
import matplotlib.pyplot as plt
import pyomo.environ as pyo
from pyomo.opt import SolverFactory
from scipy import interpolate

# starting and ending:
start_point = [0.25, 0.75]
end_point = [2.5,1.25]
#Obstacle
obstacle1 = np.array([[0.5, 0.5], [0.5, 1], [1, 1], [1, 0.5], [0.5, 0.5]])
obstacle2 = np.array([[1.5, 1], [1.5, 1.5], [2, 1.5], [2, 1], [1.5, 1]])
# obstacle-free zones:
m = 3
# Zone 1:  0<=x<=0.5, 0<=y<=2
A1=[[-1, 0], [1, 0], [0, -1], [0, 1]]
B1 = [0, 0.5, 0, 2]
# Zone 2:  0<=x<=3, 0<=y<=0.5,
A2 = [[-1, 0], [1, 0], [0, -1], [0, 1]]
B2 = [0, 3, 0, 0.5]
# Zone 3: 2<=x<=3, 0<=y<=2 
A3 = [[-1, 0], [1, 0], [0, -1], [0, 1]]
B3 = [-2, 3, 0, 2]

# degree of Bspline
d = 2
# number of control points
npoint = m + (m-1) * d

model = pyo.ConcreteModel()
model.d = pyo.Param(initialize=d)
model.constraints = pyo.ConstraintList()
model.x = pyo.Var(range(npoint)) # 0, 1, ..., npoint-1
model.y = pyo.Var(range(npoint)) # 0, 1, ..., npoint-1
# first point- initial condition
model.constraints.add(model.x[0] == start_point[0])
model.constraints.add(model.y[0] == start_point[1])
# end point - reference
model.constraints.add(model.x[npoint-1] == end_point[0])
model.constraints.add(model.y[npoint-1] == end_point[1])

# d point in transition zone (1 & 2):
for i in range(d):
    for j in range(len(B1)):
        lhs = A1[j][0] * model.x[i+1] + A1[j][1] * model.y[i+1]
        model.constraints.add(lhs <= B1[j])
    for j in range(len(B2)):
        lhs = A2[j][0] * model.x[i+1] + A2[j][1] * model.y[i+1]
        model.constraints.add(lhs <= B2[j])

# 1 point in 2nd zone:
for j in range(len(B2)):
    lhs = A2[j][0] * model.x[d+1] + A2[j][1] * model.y[d+1]
    model.constraints.add(lhs <= B2[j]) 
# d point in transition zone (2 & 3):
for i in range(d):
    for j in range(len(B2)):
        lhs = A2[j][0] * model.x[i+d+2] + A2[j][1] * model.y[i+d+2]
        model.constraints.add(lhs <= B2[j])
    for j in range(len(B3)):
        lhs = A3[j][0] * model.x[i+d+2] + A3[j][1] * model.y[i+d+2]
        model.constraints.add(lhs <= B3[j])


def _obj(model):
    cost= 0
    n = len(model.x)
    for i in range(n):
        cost += (model.y[i])**2 
    return cost

model.obj = pyo.Objective(rule=_obj, sense=pyo.minimize)
SolverFactory('ipopt').solve(model)

ctrl_x=[]
ctrl_y=[]
for i in range(npoint):
    ctrl_x.append(model.x[i].value)
    ctrl_y.append(model.y[i].value)

knot_vector=np.linspace(0,1,npoint+1-d,endpoint=True)
knot_vector=np.append([0]*d,knot_vector)
knot_vector=np.append(knot_vector,[1]*d)
tck=[knot_vector,[ctrl_x, ctrl_y], d] # define a Bspline function as required by scipy
# calculate Bspline 
t=np.linspace(0,1,(max(npoint*2,100)),endpoint=True)
out = interpolate.splev(t,tck)

# plotting
plt.figure()
plt.plot(obstacle1[0:2,0],obstacle1[0:2,1],'red',linewidth=2.0)
plt.plot(obstacle1[1:3,0],obstacle1[1:3,1],'green',linewidth=2.0)
plt.plot(obstacle1[2:4,0],obstacle1[2:4,1],'blue',linewidth=2.0)
plt.plot(obstacle1[3:5,0],obstacle1[3:5,1],'black',linewidth=2.0)
plt.plot(obstacle2[0:2,0],obstacle2[0:2,1],'red',linewidth=2.0)
plt.plot(obstacle2[1:3,0],obstacle2[1:3,1],'green',linewidth=2.0)
plt.plot(obstacle2[2:4,0],obstacle2[2:4,1],'blue',linewidth=2.0)
plt.plot(obstacle2[3:5,0],obstacle2[3:5,1],'black',linewidth=2.0)
plt.scatter(ctrl_x, ctrl_y, color='gray', s= 70, label='Control points')
plt.plot(out[0],out[1],'magenta',linewidth=2.0,label='Second-order B-spline curve')
plt.legend(loc='best')
plt.axis([0,3,-0.1,2])