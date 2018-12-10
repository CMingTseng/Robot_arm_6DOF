#!/usr/bin/env python3

import numpy as np
import os
import time

import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt

#               alpha      a           d       theta
DH = np.array([[  0.,      0.,       373.,         5.],
               [-90.,      0.,         0.,         5.],
               [  0.,    340.,         0.,         5.],
               [-90.,      0.,       338.,         5.],
               [ 90.,      0.,         0.,         5.],
               [-90.,      0.,         0.,         5.]])
print(s[1,0])
#######################################################
def rot_x(q):
    r_x = np.array([[ 1,         0,          0],
                    [ 0, np.cos(q), -np.sin(q)],
                    [ 0, np.sin(q),  np.cos(q)]])
    return r_x

def rot_y(q):
    r_y = np.array([[  np.cos(q), 0, np.sin(q)],
                    [          0, 1,         0],
                    [ -np.sin(q), 0, np.cos(q)]])
    return r_y

def rot_z(q):
    r_z = np.array([[ np.cos(q), -np.sin(q), 0],
                    [ np.sin(q),  np.cos(q), 0],
                    [         0,          0, 1]])
    return r_z

def de2ra(q):
    return q*(np.pi/180)
#######################################################
# unit millimeter
px = 100
py = 0.0
pz = 100
# unit radian
roll = 90
pitch = 0
yaw = 0

roll = de2ra(roll)
pitch = de2ra(pitch)
yaw = de2ra(yaw)

Rrpy = rot_z(yaw) * rot_y(pitch) * rot_x(roll)
lx = Rrpy[0, 0]
ly = Rrpy[1, 0]
lz = Rrpy[2, 0]


print(np.cos(np.pi))

print(rot_z(yaw)*rot_y(pitch)*rot_x(roll))


