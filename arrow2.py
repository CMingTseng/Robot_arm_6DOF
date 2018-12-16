import numpy as np
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import time

import math



class arm:
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
                self.px = 0
                self.py = 0
                self.pz = 0
                self.roll = 0
                self.pitch = 0
                self.yaw = 0
                self.runTime = 0

                self.goalF = 0

        def setGoal_W(self,Px,Py,Pz,Roll,Pitch,Yaw,RunTime):
                self.px = Px-100
                self.py = Py-100
                self.pz
                self.yaw = Yaw = Pz
                self.roll = Roll
                self.pitch = Pitch
                self.runTime = RunTime

                r_z = np.array([[ math.cos(self.yaw), -math.sin(self.yaw), 0],
                                [ math.sin(self.yaw),  math.cos(self.yaw), 0],
                                [                  0,                   0, 1]])

                r_y = np.array([[  math.cos(self.pitch), 0, math.sin(self.pitch)],
                                [                     0, 1,                    0],
                                [ -math.sin(self.pitch), 0, math.cos(self.pitch)]])

                r_x = np.array([[ 1,                   0,                    0],
                                [ 0, math.cos(self.roll), -math.sin(self.roll)],
                                [ 0, math.sin(self.roll),  math.cos(self.roll)]])

                Rrpy = r_z * r_y * r_x
                lx = Rrpy[0, 0]
                ly = Rrpy[1, 0]
                lz = Rrpy[2, 0]
                self.wx = self.px - (self.d6 + self.d5) * lx
                self.wy = self.py - (self.d6 + self.d5) * ly
                self.wz = self.pz - (self.d6 + self.d5) * lz
                self.goalF = 1
                self.wz = 35

        def FindTheta1_3(self):
                self.theta1 = math.atan(self.wy/self.wx)
                Wc0 = math.sqrt((self.wx)**2 + (self.wy)**2 + (self.wz)**2)
                Wc1 = math.sqrt((self.wx)**2 + (self.wy)**2 + (self.wz-self.d1)**2)
                #print("r= %f" %float(math.sqrt((self.wx)**2 + (self.wy)**2)))
                #print("Wc0= %f" %float(Wc0))
                #print("Wc1= %f" %float(Wc1))
                #print((self.d1**2 + Wc1**2 - Wc0**2 )/(2*self.d1*Wc1))
                phy1 = math.acos((self.d1**2 + Wc1**2 - Wc0**2 )/(2*self.d1*Wc1))
                phy2 = math.acos((Wc1**2 + self.a2**2 - self.d4**2)/(2*Wc1*self.a2))
                
                #print("phy1 = %f" %phy1)
                #print("phy2 = %f" %phy2)
                #print((math.acos((Wc0**2 - self.d1**2 - Wc1**2)/(2*self.d1*Wc1))))
                self.theta2 = (math.pi/2) - phy1 - phy2
                #print(self.theta2)
                phy3 = math.acos((self.a2**2 + self.d4**2 - Wc1**2 )/(2*self.a2*self.d4))
                self.theta3 = (math.pi/2) - phy3
                #print(self.theta3)
        #def Twto3():


if __name__ == "__main__":
        Arm = arm()
        #Arm.setGoal_W(340*math.cos(math.pi/6)+100,340*math.sin(math.pi/6)+100,35,0,0,0,1)
        Arm.setGoal_W(340*math.cos(math.pi/5)+100,340*math.sin(math.pi/5)+100,35,0,0,0,1)
        Arm.FindTheta1_3()
        #print ("d= %f" %float(Arm.theta1*(180./math.pi)))
        print ("wx = %f " %Arm.wx)
        print ("wy = %f" %Arm.wy)
        print ("wz = %f" %Arm.wz)
        print ("theta1 = %f deg" %float(Arm.theta1*(180./math.pi)))
        print ("theta2 = %f deg" %float(Arm.theta2*(180./math.pi)))
        print ("theta3 = %f deg" %float(Arm.theta3*(180./math.pi)))
        print ("time = %f" %Arm.runTime)


