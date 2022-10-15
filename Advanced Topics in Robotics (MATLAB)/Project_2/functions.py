# -*- coding: utf-8 -*-
"""
Created on Tue Dec 15 22:45:23 2020

@author: harsh
"""
import numpy as np

def dist_calculation(i,j,S,D,m,n):
    if (i+1<m):
        if (S[i+1,j]!=-1) & (D[i+1,j]>D[i,j]+1):
            D[i+1,j] = D[i,j]+1
            dist_calculation(i+1,j,S,D,m,n)
    if (j+1<n):
        if (S[i,j+1]!=-1) & (D[i,j+1]>D[i,j]+1):
            D[i,j+1]=D[i,j]+1
            dist_calculation(i,j+1,S,D,m,n)
    if (i-1>-1):
        if (S[i-1,j]!=-1) & (D[i-1,j]>D[i,j]+1):
            D[i-1,j] = D[i,j]+1
            dist_calculation(i-1,j,S,D,m,n)
    if (j-1>-1):
        if (S[i,j-1]!=-1) & (D[i,j-1]>D[i,j]+1):
            D[i,j-1]=D[i,j]+1
            dist_calculation(i,j-1,S,D,m,n)
    return(D)

def shortest_path(i,j,D,m,n):
    path = np.array([[i],[j]])
    while (i!=0) & (j!=0):
        prev_node = 0          
        if (i-1>-1) & (prev_node == 0):
            if (D[i-1,j] == D[i,j]-1):
                path = np.append(path,[[i-1],[j]],axis =1)
                prev_node = 1
                i = i-1
        if (j-1>-1) & (prev_node == 0):
            if (D[i,j-1] == D[i,j]-1):
                path = np.append(path,[[i],[j-1]],axis =1)
                prev_node = 1
                j = j-1   
        if (i+1<m) & (prev_node == 0):
            if (D[i+1,j] == D[i,j]-1):
                path = np.append(path,[[i+1],[j]],axis =1)
                prev_node = 1
                i = i+1
        if (j+1<n):
            if (D[i,j+1] == D[i,j]-1):
                path = np.append(path,[[i],[j+1]],axis =1)
                j = j+1
    
    path = np.append(path,[[0],[0]],axis =1)
    path = np.flip(path,axis = 1)
    return (path)
        
    
