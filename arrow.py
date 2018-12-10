from mpl_toolkits.mplot3d import Axes3D

import time

import matplotlib
matplotlib.use("TkAgg")
from matplotlib import pyplot as plt

fig = plt.figure()
ax = fig.gca(projection='3d')
ax.set_aspect("equal")

a = Arrow3D([0,1],[0,1],[0,1], mutation_scale=20, lw=1, arrowstyle="-|>", color="k")
ax.add_artist(a)
plt.show()
time.sleep(10)