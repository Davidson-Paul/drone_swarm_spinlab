import numpy as np
import math
import matplotlib.pyplot as plt
import scipy.signal as sig
from pole_zero_plot import zplane 

def dtft(x,x_zero_loc,w):
    #x_zero_loc=0 mean n=0
    l=np.size(x)
    X=np.zeros(np.size(w))
    for n in range (-x_zero_loc,l-x_zero_loc):
        # print(x[n+x_zero_loc]**(-1j*n*w))
        X=X+x[n+x_zero_loc]*np.exp(-1j*n*w)
    return X

# n=np.arange(0,31)
# x=np.sin(2*math.pi*10*n/100)
# w=np.arange(0,2*math.pi,0.01)
# X=dtft(x,0,w)
# print(X)
# print(X.shape,w.shape)

# Xk=np.fft.fft(x)

# plt.figure()
# plt.stem(n,x)

# plt.figure()
# plt.plot(w,np.abs(X))

# plt.figure()
# plt.stem(n,np.abs(Xk))

w=np.arange(0,math.pi,0.001)
x = np.genfromtxt('/home/davidson/Desktop/surge/spinlab_swarm/leader_follower_mission/outdoor_testing/yaw_data.csv', delimiter=',', dtype=float)
n = np.arange(0,x.shape[0])

x = x[:,0]
# x = 83.0*np.ones(540)
# X = dtft(x,0,w)
# Xk = np.fft.fft(x)
# plt.figure()
# plt.plot(n,x)
# plt.figure()
# plt.plot(w,abs(X))
# plt.figure()
# plt.plot(n,abs(Xk))

# #Trying Butterworth filter
# [order,wn]=sig.buttord(0.03/math.pi,0.06/math.pi,2,40)
# print(order,wn)
# [b,a]=sig.butter(order,wn)

# #Trying Chebshev-2 filter
# [order,wn]=sig.cheb2ord(0.02/math.pi,0.04/math.pi,2,40)
# print(order,wn)
# [b,a]=sig.cheby2(order,40,wn)

#general design
[b,a]=sig.iirdesign(0.02/math.pi,0.045/math.pi,2,40,ftype='cheb2')
print(b)
print(a)
zplane(b,a)

w,h=sig.freqz(b,a,w)
w,gd=sig.group_delay((b, a),w)

fig, (ax1,ax2,ax3,ax4) = plt.subplots(4, 1, sharex=True)

ax1.plot(w,abs(h))
ax1.set_ylabel("Magnitude")

ax2.plot(w,20*np.log10(abs(h)))
ax2.set_ylabel("Magnitude in dB")

ax3.plot(w,np.angle(h))
ax3.set_ylabel("Angle")

ax4.plot(w,gd)
ax4.set_ylabel("Group Delay")

ax4.set_xlabel('Digital frequency')
fig.suptitle('Filter Frequency Response')


#Filtered output
y=sig.filtfilt(b,a,x)
plt.figure()
plt.plot(np.arange(0,np.size(y)),y,label='Filtered Data')
plt.plot(n,x,label='Sensor Data')
plt.ylabel("yaw")
plt.legend()


plt.show()

