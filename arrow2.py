import numpy as np
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import time

import math



class arm():
        def __init__(self):
                #                alpha      a           d       theta
                DH =  np.array([[  0.,      0.,       373.,         0.],
                                [-90.,      0.,         0.,         0.],
                                [  0.,    340.,         0.,         0.],
                                [-90.,      0.,       338.,         0.],
                                [ 90.,      0.,         0.,         0.],
                                [-90.,      0.,         0.,         0.]])
                self.alpha0 = DH[0,0]
                self.alpha1 = DH[1,0]
                self.alpha2 = DH[2,0]
                self.alpha3 = DH[3,0]
                self.alpha4 = DH[4,0]
                self.alpha5 = DH[5,0]

                self.a0 = DH[0,1]
                self.a1 = DH[1,1]
                self.a2 = DH[2,1]
                self.a3 = DH[3,1]
                self.a4 = DH[4,1]
                self.a5 = DH[5,1]

                self.d1 = DH[0,2]
                self.d2 = DH[1,2]
                self.d3 = DH[2,2]
                self.d4 = DH[3,2]
                self.d5 = DH[4,2]
                self.d6 = DH[5,2]

                self.theta1 = 0
                self.theta2 = 0
                self.theta3 = 0
                self.theta4 = 0
                self.theta5 = 0
                self.theta6 = 0

                self.wx = 0
                self.wy = 0
                self.wz = 0

                self.goalF = 0

        def setGoal(self,Px,Py,Pz,Roll,Pitch,Yaw):
                px = Px
                py = Py
                pz = Pz
                roll = Roll
                pitch = Pitch
                yaw = Yaw

                r_z = np.array([[ cos(yaw), -sin(yaw), 0],
                                [ sin(yaw),  cos(yaw), 0],
                                [        0,         0, 1]])

                r_y = np.array([[  cos(pitch), 0, sin(pitch)],
                                [           0, 1,          0],
                                [ -sin(pitch), 0, cos(pitch)]])

                r_x = np.array([[ 1,         0,          0],
                                [ 0, cos(roll), -sin(roll)],
                                [ 0, sin(roll),  cos(roll)]])

                Rrpy = rot_z(yaw) * rot_y(pitch) * rot_x(roll)
                lx = Rrpy[0, 0]
                ly = Rrpy[1, 0]
                lz = Rrpy[2, 0]
                self.wx = px - (d6 + d5) * lx
                self.wy = py - (d6 + d5) * ly
                self.wz = pz - (d6 + d5) * lz
                self.goalF = 1

        def Twto3():



#plot
plt.ion()
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

ax.set_xlim(0, 600)
ax.set_ylim(0, 600)
ax.set_zlim(0, 600)
ax.set_xlabel('X axis')
ax.set_ylabel('Y axis')
ax.set_zlabel('Z axis')

point = np.array([[100 , 100 , 350 , 350 , 350 ],
                  [100 , 100 , 100 , 100 , 100 ],
                  [000 , 373 , 373 , 35  , 15  ]])



#########################################################################################################


def rot_z(q):
    r_z = np.array([[ np.cos(q), -np.sin(q), 0],
                    [ np.sin(q),  np.cos(q), 0],
                    [         0,          0, 1]])
    return r_z



#print(point)
for k in range(1):
        #point = np.dot(rot_z(np.pi/60),point)

        X = point[0][:]
        Y = point[1][:]
        Z = np.array([point[2][:]])

        for i in range(len(X)):
                text = "(%d %d %d)" % (X[i],Y[i],Z[0][i])
                ax.text(X[i]+0.1, Y[i]+0.2, Z[0][i]+0.2,text,fontsize=8) 
        ax.plot_wireframe(X, Y, Z, rstride=10, cstride=10)
        ax.scatter(X, Y, Z, c='b', marker='o')

        plt.show()
        time.sleep(0.5)
        print(point)
#ax.text(X, Y, Z, 'rer',fontsize=12)
#Axes3D.text(0.5, 0.5, 0.5, s, zdir=None, **kwargs)
plt.show(block=True)


