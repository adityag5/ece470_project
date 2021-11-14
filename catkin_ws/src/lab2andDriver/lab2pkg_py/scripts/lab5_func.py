#!/usr/bin/env python
import numpy as np
from scipy.linalg import expm, logm
from lab5_header import *
import math

"""
Use 'expm' for matrix exponential.
Angles are in radian, distance are in meters.
"""

def skew_sym(S):
	w = S[0][0]
	v = S[1][0]

	S_bracket = np.array([[0,-w[2],w[1],v[0]], [w[2],0,-w[0],v[1]],
	[-w[1],w[0],0,v[2]], [0,0,0,0]])

	return S_bracket

def Get_MS():
	# =================== Your code starts here ====================#
	# Fill in the correct values for a1~6 and q1~6, as well as the M matrix
	w1 = np.array([0,0,1])
	q1 = np.array([-150,150,10],dtype=float)* 1/1000
	v1 = np.cross(-w1,q1)
	S1 = [[w1],[v1]]

	w2 = np.array([0,1,0])	
	q2 = np.array([-150,150 + 120,10 + 152],dtype=float) * 1/1000
	v2 = np.cross(-w2,q2)
	S2 = [[w2],[v2]]

	w3 = np.array([0,1,0])
	q3 = np.array([-150 + 244,150 + 120,10 + 152],dtype=float) * 1/1000
	v3 = np.cross(-w3,q3)
	S3 = [[w3],[v3]]

	w4 = np.array([0,1,0])
	q4 = np.array([-150 + 244 + 213,150 + 120 - 93,10 + 152],dtype=float) * 1/1000
	v4 = np.cross(-w4,q4)
	S4 = [[w4],[v4]]

	w5 = np.array([1,0,0])
	q5 = np.array([-150 + 244 + 213,150 + 120 - 93 + 83,10 + 152],dtype=float) * 1/1000
	v5 = np.cross(-w5,q5)
	S5 = [[w5],[v5]]

	w6 = np.array([0,1,0])
	q6 = np.array([-150 + 244 + 213 + 83,150 + 120 - 93 + 83,10 + 152],dtype=float) * 1/1000
	v6 = np.cross(-w6,q6)
	S6 = [[w6],[v6]]

	p1 = float(-150 + 244 + 213 + 83)/1000
	p2 = float(150 + 120 - 93 + 83 + 82 + 59)/1000
	p3 = float(10 + 152 + 53.5)/1000

	M = np.array([[0,-1,0,p1],[0,0,-1,p2],[1,0,0,p3],[0,0,0,1]])
	S = [S1,S2,S3,S4,S5,S6]
	# ==============================================================#
	return M, S

"""
Function that calculates encoder numbers for each motor
"""
def lab_fk(theta1, theta2, theta3, theta4, theta5, theta6):

	# Initialize the return_value 
	return_value = [None, None, None, None, None, None]

	print("Foward kinematics calculated:\n")

	# =================== Your code starts here ====================#
	M, S = Get_MS()

	SS1 = skew_sym(S[0])
	SS2 = skew_sym(S[1])
	SS3 = skew_sym(S[2])
	SS4 = skew_sym(S[3])
	SS5 = skew_sym(S[4])
	SS6 = skew_sym(S[5])

	SSl = [SS1,SS2,SS3,SS4,SS5,SS6]
	theta = [theta1, theta2, theta3, theta4, theta5, theta6]
	T = np.eye(4)

	for i in range(0,6):
		
		T = np.matmul(T,expm(SSl[i] * theta[i]))
		
	T = np.matmul(T,M)

	# ==============================================================#

	return_value[0] = theta1 + PI
	return_value[1] = theta2
	return_value[2] = theta3
	return_value[3] = theta4 - (0.5*PI)
	return_value[4] = theta5
	return_value[5] = theta6

	return return_value


"""
Function that calculates an elbow up Inverse Kinematic solution for the UR3
"""
def lab_invk(xWgrip, yWgrip, zWgrip, yaw_WgripDegree):
	# Define Link Lengths (m) and given angles 
	L1 = 0.152
	L2 = 0.12
	L3 = 0.244
	L4 = 0.093
	L5 = 0.213
	L6 = 0.083
	L7 = 0.083
	L8 = 0.082
	L9 = 0.0535
	L10 = 0.059
	d_end_L6 = 0.027
	theta_5 = np.radians(-90)

	# Convert World frame to base frame
	xgrip = xWgrip + 0.15
	ygrip = yWgrip - 0.15
	zgrip = zWgrip - 0.01
	yaw = np.radians(yaw_WgripDegree)

	# Solve for wrist center point with xgrip,ygrip,zgrip,yaw
	x_s = L9*np.cos(yaw)
	y_s = L9*np.sin(yaw)
	x_cen = xgrip - x_s
	y_cen = ygrip - y_s
	z_cen = zgrip

	# Solve for theta1
	phi = math.atan2(y_cen,x_cen)
	h = math.sqrt(x_cen**2  + y_cen**2)
	psi = math.asin((d_end_L6 + L6) / h)
	theta_1 = phi - psi

	#Solve for theta6
	theta_6 = theta_1 + np.radians(90) - yaw

	# Find x3end, y3end, z3end
	x3end = x_cen - ((L7)*math.cos(theta_1)) + ((d_end_L6 + L6)*math.sin(theta_1))
	y3end = y_cen - ((L7)*math.sin(theta_1)) - ((d_end_L6 + L6)*math.cos(theta_1))
	z3end = z_cen + L10 + L8

	z_i = z3end - L1
	x_i = math.sqrt(x3end**2 + y3end**2)

	alpha = math.atan2(z_i,x_i)
	s_i = math.sqrt(x_i**2 + z_i**2)
	print(s_i, L3, L5, x_i, z_i)
	print((L3**2 + L5**2 - s_i**2) / (2*L3*L5))
	beta = math.acos((L3**2 + L5**2 - s_i**2) / (2*L3*L5))
	gamma = math.acos((L3**2 + s_i**2 - L5**2) / (2*L3*s_i))
	theta_2 = -alpha - gamma  	
	theta_3 = np.radians(180) - beta
	theta_4 = -theta_2 - theta_3
	print(np.degrees([theta_1, theta_2, theta_3, theta_4, theta_5, theta_6]))
	return lab_fk(theta_1, theta_2, theta_3, theta_4, theta_5, theta_6)
	# ==============================================================#
   


