#Script to solve consensus tracking by solving ODE in centralised manner

import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp
import math
import time

a1=0
a2=0
a3=0
a4=0
a5=0
adj = np.loadtxt('/home/davidson/catkin_ws/src/iq_gnc/adjacency_matrix.txt', dtype=int)

def equations(t,y,v):
    global a1,a2,a3,a4,a5

    # yr=20*np.sin(2*math.pi*1/160*t)
    # v=20*(2*math.pi*1/160)*np.cos(2*math.pi*1/160*t)
    # a=-20*(2*math.pi*1/160)**2*np.sin(2*math.pi*1/160*t)
    yr=10*t
    vr=10
    ar=0

    y1=y[0]
    y2=y[1]
    y3=y[2]
    y4=y[3]
    y5=y[4]

    v1=v[0]
    v2=v[1]
    v3=v[2]
    v4=v[3]
    v5=v[4]

    A=adj[0,:]
    n=np.sum(A)
    ddy1 = 1/n*(A[0]*(a1-(y1-y1)-(v1-v1))
                +A[1]*(a2-(y1-y2)-(v1-v2))
                +A[2]*(a3-(y1-y3)-(v1-v3))
                +A[3]*(a4-(y1-y4)-(v1-v4))
                +A[4]*(a5-(y1-y5)-(v1-v5))
                +A[5]*(ar-(y1-yr)-(v1-vr)))
    
    A=adj[1,:]
    n=np.sum(A)
    ddy2 = 1/n*(A[0]*(a1-(y2-y1)-(v2-v1))
                +A[1]*(a2-(y2-y2)-(v2-v2))
                +A[2]*(a3-(y2-y3)-(v2-v3))
                +A[3]*(a4-(y2-y4)-(v2-v4))
                +A[4]*(a5-(y2-y5)-(v2-v5))
                +A[5]*(ar-(y2-yr)-(v2-vr)))
    
    A=adj[2,:]
    n=np.sum(A)
    ddy3 = 1/n*(A[0]*(a1-(y3-y1)-(v3-v1))
                +A[1]*(a2-(y3-y2)-(v3-v2))
                +A[2]*(a3-(y3-y3)-(v3-v3))
                +A[3]*(a4-(y3-y4)-(v3-v4))
                +A[4]*(a5-(y3-y5)-(v3-v5))
                +A[5]*(ar-(y3-yr)-(v3-vr)))
    
    A=adj[3,:]
    n=np.sum(A)
    ddy4 = 1/n*(A[0]*(a1-(y4-y1)-(v4-v1))
                +A[1]*(a2-(y4-y2)-(v4-v2))
                +A[2]*(a3-(y4-y3)-(v4-v3))
                +A[3]*(a4-(y4-y4)-(v4-v4))
                +A[4]*(a5-(y4-y5)-(v4-v5))
                +A[5]*(ar-(y4-yr)-(v4-vr)))
    
    A=adj[4,:]
    n=np.sum(A)
    ddy5 = 1/n*(A[0]*(a1-(y5-y1)-(v5-v1))
                +A[1]*(a2-(y5-y2)-(v5-v2))
                +A[2]*(a3-(y5-y3)-(v5-v3))
                +A[3]*(a4-(y5-y4)-(v5-v4))
                +A[4]*(a5-(y5-y5)-(v5-v5))
                +A[5]*(ar-(y5-yr)-(v5-vr)))

    dy1=v1
    dy2=v2
    dy3=v3
    dy4=v4
    dy5=v5

    a1=ddy1
    a2=ddy2
    a3=ddy3
    a4=ddy4
    a5=ddy5

    return [a1,a2,a3,a4,a5]

def rk_4(eq,u0,t0,tf,dt):
    # print(t0," ",tf)
    # N = int(np.ceil((tf-t0)/dt)) #(no.of samples-1)
    N = 1
    x = np.array(u0[0:5])
    v = np.array(u0[5:10])
    # print(type(x))
    # print(type(v))
    
    k1 = v
    k2 = np.array(eq(t0,x,v))
    # print(type(k2))

    s1 = v+k2*dt/2
    s2 = np.array(eq(t0+dt/2,x+k1*dt/2,v+k2*dt/2))

    l1 = v+s2*dt/2
    l2 = np.array(eq(t0+dt/2,x+s1*dt/2,v+s2*dt/2))

    p1 = v+l2*dt
    p2 = np.array(eq(t0+dt,x+l1*dt,v+l2*dt))

    x1 = x + (k1+2*s1+2*l1+p1)*dt/6
    v1 = v + (k2+2*s2+2*l2+p2)*dt/6

    print("t:",t0)
    print("x:",x)
    print("v:",v)
    return x1,v1

# Time span
t_start = 0
t_end = 50
dt = 0.2

# Initialize empty lists to store the results
t_points = []
y1_points = []
y2_points = []
y3_points = []
y4_points = []
y5_points = []

v1_points = []
v2_points = []
v3_points = []
v4_points = []
v5_points = []

# Initialize time and initial conditions
t = t_start
ini = np.array([0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0])

# Solve the system of equations continuously using a while loop
while t <= t_end:
    # t1=time.time()
    adj = np.loadtxt('/home/davidson/catkin_ws/src/iq_gnc/adjacency_matrix.txt', dtype=int)

    t_points.append(t)
    y1_points.append(ini[0])
    y2_points.append(ini[1])
    y3_points.append(ini[2])
    y4_points.append(ini[3])
    y5_points.append(ini[4])

    v1_points.append(ini[5])
    v2_points.append(ini[6])
    v3_points.append(ini[7])
    v4_points.append(ini[8])
    v5_points.append(ini[9])

    # Perform one integration step
    x,v = rk_4(equations,ini,t,t+dt,dt)
    t += dt

    # Update initial conditions
    ini = [x[0],x[1],x[2],x[3],x[4],v[0],v[1],v[2],v[3],v[4]]

    # t2=time.time()
    # print(t2-t1)
    time.sleep(0.004)

# Convert lists to arrays
t_points = np.array(t_points)

y1_points = np.array(y1_points)
y2_points = np.array(y2_points)
y3_points = np.array(y3_points)
y4_points = np.array(y4_points)
y5_points = np.array(y5_points)

v1_points = np.array(v1_points)
v2_points = np.array(v2_points)
v3_points = np.array(v3_points)
v4_points = np.array(v4_points)
v5_points = np.array(v5_points)

# Plot the variables
plt.figure(figsize=(10, 6))
plt.plot(t_points, y1_points, label='y1')
plt.plot(t_points, y2_points, label='y2')
plt.plot(t_points, y3_points, label='y3')
plt.plot(t_points, y4_points, label='y4')
plt.plot(t_points, y5_points, label='y5')

plt.plot(t_points, v1_points, label='v1')
plt.plot(t_points, v2_points, label='v2')
plt.plot(t_points, v3_points, label='v3')
plt.plot(t_points, v4_points, label='v4')
plt.plot(t_points, v5_points, label='v5')

plt.xlabel('Time')
plt.ylabel('Variables')
plt.legend()
plt.grid(True)
plt.title('System of Second-Order Differential Equations')
plt.show()