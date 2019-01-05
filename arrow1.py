import numpy as np
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.animation as animation
import time

#plot
#plt.ion()
fig = plt.figure(figsize=(10,60))
plt.subplots_adjust(left=0, bottom=0, right=1, top=1)
#plt.autoscale(enable=False, axis='both', tight=None)
ax = fig.add_subplot(111, projection='3d')
ax.autoscale(enable=True, axis='both', tight=True)

ax.set_xlim(0, 600)
ax.set_ylim(0, 600)
ax.set_zlim(0, 600)
ax.set_xlabel('X axis')
ax.set_ylabel('Y axis')
ax.set_zlabel('Z axis')

point = np.array([[  0 , 100 , 350 , 350 , 350 ],
                  [  0 , 100 , 100 , 100 , 100 ],
                  [000 , 373 , 373 , 35  , 15  ]])



#########################################################################################################


def rot_z(q):
    r_z = np.array([[ np.cos(q), -np.sin(q), 0],
                    [ np.sin(q),  np.cos(q), 0],
                    [         0,          0, 1]])
    return r_z



#print(point)
def anim(i):
        global point
        point = np.dot(rot_z(np.pi/1000),point)

        X = point[0][:]
        Y = point[1][:]
        Z = np.array([point[2][:]])
        ax.clear()
        for i in range(len(X)):
                text = "(%.2f ,%.2f , %.2f)" % (X[i],Y[i],Z[0][i])
                ax.text(X[i]+0.1, Y[i]+0.2, Z[0][i]+0.2,text,fontsize=8) 
        ax.plot_wireframe(X, Y, Z, rstride=10, cstride=10)
        #ax.clear()
        ax.scatter(X, Y, Z, c='b', marker='o')


ani = animation.FuncAnimation(fig=fig, func=anim, frames=1000, init_func=None,
                                interval=10, blit=False)
#plt.show(block=True)
plt.show()


