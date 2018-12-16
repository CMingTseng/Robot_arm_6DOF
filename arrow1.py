import numpy as np
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import time

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


