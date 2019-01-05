import numpy as np
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import time

<<<<<<< HEAD
#plot
=======
>>>>>>> 1a6dae9... plot OK
plt.ion()
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

ax.set_xlim(0, 600)
ax.set_ylim(0, 600)
ax.set_zlim(0, 600)
ax.set_xlabel('X axis')
ax.set_ylabel('Y axis')
ax.set_zlabel('Z axis')

<<<<<<< HEAD
point = np.array([[100 , 100 , 350 , 350 , 350 ],
                  [100 , 100 , 100 , 100 , 100 ],
                  [000 , 373 , 373 , 35  , 15  ]])

#plot


#               alpha      a           d       theta
DH = np.array([[  0.,      0.,       373.,         0.],
               [-90.,      0.,         0.,         0.],
               [  0.,    340.,         0.,         0.],
               [-90.,      0.,       338.,         0.],
               [ 90.,      0.,         0.,         0.],
               [-90.,      0.,         0.,         0.]])


#########################################################################################################

def init():
        theta1 = 0
        theta2 = 0
        theta3 = 0
        theta4 = 0
        theta5 = 0
        theta6 = 0


=======
>>>>>>> 1a6dae9... plot OK
def rot_z(q):
    r_z = np.array([[ np.cos(q), -np.sin(q), 0],
                    [ np.sin(q),  np.cos(q), 0],
                    [         0,          0, 1]])
    return r_z

<<<<<<< HEAD


Tw_0 =np.array([[               1,               0,             0,             10],
                [               0,               1,             0,             10],
                [               0,               0,             1,              0],
                [               0,               0,             0,              1]])

T0_1 =np.array([[  np.cos(theta1), -np.sin(theta1),             0,             10],
                [  np.sin(theta1),  np.cos(theta1),             0,             10],
                [               0,               0,             1,              0],
                [               0,               0,             0,              1]])


#########################################################################################################


=======
point = np.array([[100 , 100 , 350 , 350 , 350 ],
                  [100 , 100 , 100 , 100 , 100 ],
                  [000 , 373 , 373 , 35  , 15  ]])
#print(point)

#point = np.dot(rot_z(np.pi/6),point)
>>>>>>> 1a6dae9... plot OK

#print(point)
for k in range(10):
        point = np.dot(rot_z(np.pi/60),point)

        X = point[0][:]
        Y = point[1][:]
        Z = np.array([point[2][:]])

        for i in range(len(X)):
                text = "(%d %d %d)" % (X[i],Y[i],Z[0][i])
                ax.text(X[i]+0.1, Y[i]+0.2, Z[0][i]+0.2,text,fontsize=8) 
        ax.plot_wireframe(X, Y, Z, rstride=10, cstride=10)
        ax.scatter(X, Y, Z, c='b', marker='o')
<<<<<<< HEAD

        plt.show()
        time.sleep(0.5)
        print(point)
#ax.text(X, Y, Z, 'rer',fontsize=12)
#Axes3D.text(0.5, 0.5, 0.5, s, zdir=None, **kwargs)
plt.show(block=True)




class arm(dh):
        def __init__(self,dh):
                
=======

        plt.show()
        time.sleep(0.5)
        print(point)
#ax.text(X, Y, Z, 'rer',fontsize=12)
#Axes3D.text(0.5, 0.5, 0.5, s, zdir=None, **kwargs)
plt.show(block=True)
>>>>>>> 1a6dae9... plot OK
