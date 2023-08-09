#This script is for generating different time-dependent paths

import numpy as np
import math

def generate_circle_path(t, r, vel, h):
    w = vel/r
    xr = np.array([r*math.cos(w*t), r*math.sin(w*t), h])
    vr = np.array([-r*w*math.sin(w*t), r*w*math.cos(w*t), 0])
    ar = np.array([-r*w**2*math.cos(w*t), -r*w**2*math.sin(w*t), 0])
    return xr,vr,ar

def generate_infinity_path(t, length=3, width=5):
    x = length * np.sin(t)
    y = width * np.sin(t) * np.cos(t)
    return x, y

def generate_sine_path(t,amp,fc, h, v):
    xr=np.array([4*t,amp*math.sin(2*math.pi*fc*t),h])
    vr=np.array([4*v,amp*2*math.pi*fc*math.cos(2*math.pi*fc*t),0])
    ar=np.array([0,-amp*(2*math.pi*fc)**2*math.sin(2*math.pi*fc*t),0])
    yr=math.atan2(vr[1],vr[0])
    wr=(vr[0]*ar[1]-vr[1]*ar[0])/(vr[0]**2+vr[1]**2)
    return xr,vr,ar,yr,wr

def give_setpoint(x,y,z):
    xr=np.array([x,y,z])
    vr=np.array([0,0,0])
    ar=np.array([0,0,0])
    yr=0
    wr=0
    return xr,vr,ar,yr,wr

def generate_straight_path(t,h,v):
    xr=np.array([4*t,0,h])
    vr=np.array([4*v,0,0])
    ar=np.array([0,0,0])
    yr=math.atan2(vr[1],vr[0])
    wr=(vr[0]*ar[1]-vr[1]*ar[0])/(vr[0]**2+vr[1]**2)
    return xr,vr,ar,yr,wr