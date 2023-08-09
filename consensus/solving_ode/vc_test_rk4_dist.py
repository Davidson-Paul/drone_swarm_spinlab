#Script to solve consensus tracking by solving ODE in distributed manner

import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp
import math
import threading
import time

y1=0
y2=0
y3=0
y4=0
y5=0

v1=0
v2=0
v3=0
v4=0
v5=0

a1=0
a2=0
a3=0
a4=0
a5=0

ddy1=0
ddy2=0
ddy3=0
ddy4=0
ddy5=0

adj = np.loadtxt('/home/davidson/catkin_ws/src/iq_gnc/adjacency_matrix.txt', dtype=int)

def equation_1(t,x,v):
    global y1,y2,y3,y4,y5,v1,v2,v3,v4,v5,a1,a2,a3,a4,a5,ddy1,ddy2,ddy3,ddy4,ddy5,adj
    # yr=20*np.sin(2*math.pi*1/160*t)
    # vr=20*(2*math.pi*1/160)*np.cos(2*math.pi*1/160*t)
    # ar=-20*(2*math.pi*1/160)**2*np.sin(2*math.pi*1/160*t)
    yr=10*t
    vr=10
    ar=0

    y1=x
    v1=v
    A=adj[0,:]
    n=np.sum(A)
    ddy1 = 1/n*(A[0]*(a1-(y1-y1)-(v1-v1))
                +A[1]*(a2-(y1-y2)-(v1-v2))
                +A[2]*(a3-(y1-y3)-(v1-v3))
                +A[3]*(a4-(y1-y4)-(v1-v4))
                +A[4]*(a5-(y1-y5)-(v1-v5))
                +A[5]*(ar-(y1-yr)-(v1-vr)))
    return ddy1

def equation_2(t,x,v):
    global y1,y2,y3,y4,y5,v1,v2,v3,v4,v5,a1,a2,a3,a4,a5,ddy1,ddy2,ddy3,ddy4,ddy5,adj

    # yr=20*np.sin(2*math.pi*1/160*t)
    # vr=20*(2*math.pi*1/160)*np.cos(2*math.pi*1/160*t)
    # ar=-20*(2*math.pi*1/160)**2*np.sin(2*math.pi*1/160*t)
    yr=10*t
    vr=10
    ar=0

    y2=x
    v2=v
    A=adj[1,:]
    n=np.sum(A)
    ddy2 = 1/n*(A[0]*(a1-(y2-y1)-(v2-v1))
                +A[1]*(a2-(y2-y2)-(v2-v2))
                +A[2]*(a3-(y2-y3)-(v2-v3))
                +A[3]*(a4-(y2-y4)-(v2-v4))
                +A[4]*(a5-(y2-y5)-(v2-v5))
                +A[5]*(ar-(y2-yr)-(v2-vr)))
    return ddy2

def equation_3(t,x,v):
    global y1,y2,y3,y4,y5,v1,v2,v3,v4,v5,a1,a2,a3,a4,a5,ddy1,ddy2,ddy3,ddy4,ddy5,adj

    # yr=20*np.sin(2*math.pi*1/160*t)
    # vr=20*(2*math.pi*1/160)*np.cos(2*math.pi*1/160*t)
    # ar=-20*(2*math.pi*1/160)**2*np.sin(2*math.pi*1/160*t)
    yr=10*t
    vr=10
    ar=0

    y3=x
    v3=v
    A=adj[2,:]
    n=np.sum(A)
    ddy3 = 1/n*(A[0]*(a1-(y3-y1)-(v3-v1))
                +A[1]*(a2-(y3-y2)-(v3-v2))
                +A[2]*(a3-(y3-y3)-(v3-v3))
                +A[3]*(a4-(y3-y4)-(v3-v4))
                +A[4]*(a5-(y3-y5)-(v3-v5))
                +A[5]*(ar-(y3-yr)-(v3-vr)))
    return ddy3

def equation_4(t,x,v):
    global y1,y2,y3,y4,y5,v1,v2,v3,v4,v5,a1,a2,a3,a4,a5,ddy1,ddy2,ddy3,ddy4,ddy5,adj

    # yr=20*np.sin(2*math.pi*1/160*t)
    # vr=20*(2*math.pi*1/160)*np.cos(2*math.pi*1/160*t)
    # ar=-20*(2*math.pi*1/160)**2*np.sin(2*math.pi*1/160*t)
    yr=10*t
    vr=10
    ar=0

    y4=x
    v4=v
    A=adj[3,:]
    n=np.sum(A)
    ddy4 = 1/n*(A[0]*(a1-(y4-y1)-(v4-v1))
                +A[1]*(a2-(y4-y2)-(v4-v2))
                +A[2]*(a3-(y4-y3)-(v4-v3))
                +A[3]*(a4-(y4-y4)-(v4-v4))
                +A[4]*(a5-(y4-y5)-(v4-v5))
                +A[5]*(ar-(y4-yr)-(v4-vr)))
    return ddy4

def equation_5(t,x,v):
    global y1,y2,y3,y4,y5,v1,v2,v3,v4,v5,a1,a2,a3,a4,a5,ddy1,ddy2,ddy3,ddy4,ddy5,adj

    # yr=20*np.sin(2*math.pi*1/160*t)
    # vr=20*(2*math.pi*1/160)*np.cos(2*math.pi*1/160*t)
    # ar=-20*(2*math.pi*1/160)**2*np.sin(2*math.pi*1/160*t)
    yr=10*t
    vr=10
    ar=0

    y5=x
    v5=v
    A=adj[4,:]
    n=np.sum(A)
    ddy5 = 1/n*(A[0]*(a1-(y5-y1)-(v5-v1))
                +A[1]*(a2-(y5-y2)-(v5-v2))
                +A[2]*(a3-(y5-y3)-(v5-v3))
                +A[3]*(a4-(y5-y4)-(v5-v4))
                +A[4]*(a5-(y5-y5)-(v5-v5))
                +A[5]*(ar-(y5-yr)-(v5-vr)))
    return ddy5

def forward_euler_di(eq, u0, t0, tf, dt):
    # print(t0+" "+tf)
    N = int(np.ceil((tf-t0)/dt)) #(no.of samples-1)
    # N=1
    t = np.zeros(N+1) #actually we get N+1 samples in the time interval
    x = np.zeros(N+1)
    v = np.zeros(N+1)
    a = np.zeros(N+1)
    x[0] = u0[0]
    v[0] = u0[1]
    t[0] = t0
    for n in range(N):
        a[n] = eq(t[n],x[n],v[n])

        t[n+1] = t[n] + dt
        x[n+1] = x[n] + v[n]*dt #+ 0.5*a[n]*dt**2
        v[n+1] = v[n] + a[n]*dt

    # print("N:",N)
    # print("x:",x)
    return x,v,t

def rk_4(eq,u0,t0,tf,dt):
    # print(t0," ",tf)
    N = int(np.ceil((tf-t0)/dt)) #(no.of samples-1)
    # N = 1
    t = np.zeros(N+1) #actually we get N+1 samples in the time interval
    x = np.zeros(N+1)
    v = np.zeros(N+1)
    x[0] = u0[0]
    v[0] = u0[1]
    t[0] = t0
    for n in range(N):
        k1 = v[n]
        k2 = eq(t[n],x[n],v[n])

        s1 = v[n]+k2*dt/2
        s2 = eq(t[n]+dt/2,x[n]+k1*dt/2,v[n]+k2*dt/2)

        l1 = v[n]+s2*dt/2
        l2 = eq(t[n]+dt/2,x[n]+s1*dt/2,v[n]+s2*dt/2)

        p1 = v[n]+l2*dt
        p2 = eq(t[n]+dt,x[n]+l1*dt,v[n]+l2*dt)

        t[n+1] = t[n] + dt
        x[n+1] = x[n] + (k1+2*s1+2*l1+p1)*dt/6
        v[n+1] = v[n] + (k2+2*s2+2*l2+p2)*dt/6
    return x,v,t

t1_points = []
t2_points = []
t3_points = []
t4_points = []
t5_points = []

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

# Time span
t_end = 50
dt = 0.05

def pro1():
    global y1,y2,y3,y4,y5,v1,v2,v3,v4,v5,a1,a2,a3,a4,a5,ddy1,ddy2,ddy3,ddy4,ddy5,adj
    t=0
    while t <= t_end:
        current_time = time.time()  # Get current time
        # Synchronize publishing with a specific time interval
        print(current_time)
        if (current_time*20) % 1.0 < 0.1:
            adj = np.loadtxt('/home/davidson/catkin_ws/src/iq_gnc/adjacency_matrix.txt', dtype=int)
            t1_points.append(t)
            y1_points.append(y1)
            v1_points.append(v1)
            print("t1:",t)

            # Perform one integration step
            x,v,time_steps=rk_4(equation_1,[y1,v1],t,t+dt,dt)
            t += dt

            time.sleep(0.02)
            # Update initial conditions
            y1 = x[-1]
            v1 = v[-1]
            a1 = ddy1
            # print("y1:",y1)
            # print("v1:",v1)
            # print("a1:",a1)
            time.sleep(0.02)
            # time.sleep(0.004)

def pro2():
    global y1,y2,y3,y4,y5,v1,v2,v3,v4,v5,a1,a2,a3,a4,a5,ddy1,ddy2,ddy3,ddy4,ddy5,adj
    t=0
    while t <= t_end:
        current_time = time.time()  # Get current time
        # Synchronize publishing with a specific time interval
        if (current_time*20) % 1.0 < 0.1:
            adj = np.loadtxt('/home/davidson/catkin_ws/src/iq_gnc/adjacency_matrix.txt', dtype=int)
            t2_points.append(t)
            y2_points.append(y2)
            v2_points.append(v2)
            print("t2:",t)

            # Perform one integration step
            x,v,time_steps=rk_4(equation_2,[y2,v2],t,t+dt,dt)
            t += dt

            time.sleep(0.02)
            # Update initial conditions
            y2 = x[-1]
            v2 = v[-1]
            a2 = ddy2
            # print("y2:",y2)
            # print("v2:",v2)
            # print("a2:",a2)
            time.sleep(0.02)
            # time.sleep(0.004)

def pro3():
    global y1,y2,y3,y4,y5,v1,v2,v3,v4,v5,a1,a2,a3,a4,a5,ddy1,ddy2,ddy3,ddy4,ddy5,adj
    t=0
    while t <= t_end:
        current_time = time.time()  # Get current time
        # Synchronize publishing with a specific time interval
        if (current_time*20) % 1.0 < 0.1:
            adj = np.loadtxt('/home/davidson/catkin_ws/src/iq_gnc/adjacency_matrix.txt', dtype=int)
            t3_points.append(t)
            y3_points.append(y3)
            v3_points.append(v3)
            print("t3:",t)

            # Perform one integration step
            x,v,time_steps=rk_4(equation_3,[y3,v3],t,t+dt,dt)
            t += dt

            time.sleep(0.02)
            # Update initial conditions
            y3 = x[-1]
            v3 = v[-1]
            a3 = ddy3
            # print("y3:",y3)
            # print("v3:",v3)
            # print("a3:",a3)
            time.sleep(0.02)
            # time.sleep(0.004)

def pro4():
    global y1,y2,y3,y4,y5,v1,v2,v3,v4,v5,a1,a2,a3,a4,a5,ddy1,ddy2,ddy3,ddy4,ddy5,adj
    t=0
    while t <= t_end:
        current_time = time.time()  # Get current time
        # Synchronize publishing with a specific time interval
        if (current_time*20) % 1.0 < 0.1:
            adj = np.loadtxt('/home/davidson/catkin_ws/src/iq_gnc/adjacency_matrix.txt', dtype=int)
            t4_points.append(t)
            y4_points.append(y4)
            v4_points.append(v4)
            print("t4:",t)

            # Perform one integration step
            x,v,time_steps=rk_4(equation_4,[y4,v4],t,t+dt,dt)
            t += dt

            time.sleep(0.02)
            # Update initial conditions
            y4 = x[-1]
            v4 = v[-1]
            a4 = ddy4
            # print("y4:",y4)
            # print("v4:",v4)
            # print("a4:",a4)
            time.sleep(0.02)
            # time.sleep(0.004)

def pro5():
    global y1,y2,y3,y4,y5,v1,v2,v3,v4,v5,a1,a2,a3,a4,a5,ddy1,ddy2,ddy3,ddy4,ddy5,adj
    t=0
    while t <= t_end:
        current_time = time.time()  # Get current time
        # Synchronize publishing with a specific time interval
        if (current_time*20) % 1.0 < 0.1:
            adj = np.loadtxt('/home/davidson/catkin_ws/src/iq_gnc/adjacency_matrix.txt', dtype=int)
            t5_points.append(t)
            y5_points.append(y5)
            v5_points.append(v5)
            print("t5:",t)

            # Perform one integration step
            x,v,time_steps=rk_4(equation_5,[y5,v5],t,t+dt,dt)
            t += dt

            time.sleep(0.02)
            # Update initial conditions
            y5 = x[-1]
            v5 = v[-1]
            a5 = ddy5
            # print("y5:",y5)
            # print("v5:",v5)
            # print("a5:",a5)
            time.sleep(0.02)
            # time.sleep(0.004)

thread1 = threading.Thread(target=pro1)
thread2 = threading.Thread(target=pro2)
thread3 = threading.Thread(target=pro3)
thread4 = threading.Thread(target=pro4)
thread5 = threading.Thread(target=pro5)

# Start the threads
thread1.start()
thread2.start()
thread3.start()
thread4.start()
thread5.start()

# Wait for the threads to finish
thread1.join()
thread2.join()
thread3.join()
thread4.join()
thread5.join()

# Plot the variables
plt.figure(figsize=(10, 6))
plt.plot(t1_points, y1_points, label='y1')
plt.plot(t2_points, y2_points, label='y2')
plt.plot(t3_points, y3_points, label='y3')
plt.plot(t4_points, y4_points, label='y4')
plt.plot(t5_points, y5_points, label='y5')

plt.plot(t1_points, v1_points, label='v1')
plt.plot(t2_points, v2_points, label='v2')
plt.plot(t3_points, v3_points, label='v3')
plt.plot(t4_points, v4_points, label='v4')
plt.plot(t5_points, v5_points, label='v5')

plt.xlabel('Time')
plt.ylabel('Variables')
plt.legend()
plt.grid(True)
plt.title('System of Second-Order Differential Equations')
plt.show()