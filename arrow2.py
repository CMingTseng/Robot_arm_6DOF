import numpy as np
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import time
import tf

import math



class arm:
        def __init__(self):
                #                      alpha      a (mm)      d       theta
                DH =  np.array([[        0.,      0.,       373.,         0.],
                                [-math.pi/2,      0.,         0.,         0.],
                                [        0.,    340.,         0.,         0.],
                                [-math.pi/2,      0.,       338.,         0.],
                                [ math.pi/2,      0.,         0.,         0.],
                                [-math.pi/2,      0.,         0.,         0.]])
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
                self.pz = Pz
                self.yaw = Yaw 
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

                
                off = np.array([0,-10,0])
                self.Rrpy = np.dot(r_z , np.dot(r_y,r_x))
                #print(np.dot(self.Rrpy , off.T))
                W = np.dot(r_z , np.dot(r_y , np.dot(r_x , off.T)))
                
                #print(W)
                self.wx = self.px + W[0]
                self.wy = self.py + W[1]
                self.wz = self.pz + W[2]
                self.goalF = 1
                #self.wz = 35

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
        def craig_dh_Trans(self,theta, alpha_, d, a_):#(theta_i, alpha_i-1, d_i, a_i-1)
                Ta_b = np.array([[                  math.cos(theta),                 -math.sin(theta),                 0,                  a_],
                                 [ math.sin(theta)*math.cos(alpha_), math.cos(theta)*math.cos(alpha_), -math.sin(alpha_), -math.sin(alpha_)*d],
                                 [ math.sin(theta)*math.sin(alpha_), math.cos(theta)*math.sin(alpha_),  math.cos(alpha_),  math.cos(alpha_)*d],
                                 [                                0,                                0,                 0,                   1]])
                return Ta_b

        def FindTheta4_6(self):
                T0_1 = self.craig_dh_Trans(self.theta1,self.alpha0,self.d1,self.a0)
                T1_2 = self.craig_dh_Trans(self.theta2,self.alpha1,self.d2,self.a1)
                T2_3 = self.craig_dh_Trans(self.theta3,self.alpha2,self.d3,self.a2)

                T0_3 = np.dot(T0_1,np.dot(T1_2,T2_3))
                R0_3 = T0_3[:3,:3]
                #print(R0_3)

                R6_c = np.array([[0.,-1., 0.],
                                 [0., 0.,-1.],
                                 [1., 0., 0.]])
                Rw_c = np.array([[1., 0., 0.],
                                 [0., 1., 0.],
                                 [0., 0., 1.]])
                #print(np.linalg.inv(self.Rrpy))
                R3_6 = np.dot(R0_3,np.dot(Rw_c,np.linalg.inv(self.Rrpy)))
                R3_6 = np.array([[ 0.7627 ,  0.1477 , 0.6297],
                                 [-0.6468 ,  0.1744 , 0.7424],
                                 [      0 , -0.9735 , 0.2286]])
                print(R3_6[0,1])

                print(R3_6)
                betaT  = math.atan((math.sqrt(R3_6[2,0]**2+R3_6[2,1]**2))/R3_6[2,2])
                alphaT = math.atan((R3_6[1,2]/math.sin(betaT))/(R3_6[0,2]/math.sin(betaT)))
                gammaT = math.atan((R3_6[2,1]/math.sin(betaT))/(-R3_6[2,0]/math.sin(betaT)))
                print("b= %f" %float(betaT*(180./math.pi)))
                print("a= %f" %float(alphaT*(180./math.pi)))
                print("g= %f" %float(gammaT*(180./math.pi)))

if __name__ == "__main__":
        Arm = arm()
        #Arm.setGoal_W(340*math.cos(math.pi/6)+100,340*math.sin(math.pi/6)+100,35,0,0,0,1)
        Arm.setGoal_W(500,340,35,0,0,0,1)
        Arm.FindTheta1_3()
        Arm.FindTheta4_6()
        #print ("d= %f" %float(Arm.theta1*(180./math.pi)))
        print ("px = %f " %(Arm.px+100.))
        print ("py = %f " %(Arm.py+100.))
        print ("pz = %f " %Arm.pz)
        print ("wx = %f " %(Arm.wx+100.))
        print ("wy = %f " %(Arm.wy+100.))
        print ("wz = %f " %Arm.wz)
        print ("theta1 = %f deg" %float(Arm.theta1*(180./math.pi)))
        print ("theta2 = %f deg" %float(Arm.theta2*(180./math.pi)))
        print ("theta3 = %f deg" %float(Arm.theta3*(180./math.pi)))
        print ("time = %f" %Arm.runTime)


