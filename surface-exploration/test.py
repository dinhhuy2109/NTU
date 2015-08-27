import numpy as np
import scipy as sp

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm

import time


N = 100
s = np.array([3 ,5]) # where the bird is hiding

n = 2*np.random.randn(2,N)
x = np.zeros((2,N))

plt.ion()
fig = plt.figure()
ax = fig.add_subplot(121) 
ax.axis([0,10, 0,10 ])
ax.set_ylabel('Y LABEL')
ax.set_xlabel('X LABEL')
ax.scatter(s[0], s[1], c = 'r', marker = 'o')
x[0,:] = n[0,:] +s[0]
x[1,:] = n[1,:]+s[1]
ax.scatter(x[0,:],x[1,:] , c = 'y', marker  = '.')
ax.grid(True)

Sa = np.arange(2,4,0.05)
Sb = np.arange(4,6,0.05)

L = Sa.size

Pr = np.ones([L, L])
Po = np.ones([L, L])

Pr = Pr/sum(sum(Pr))
Po = Po/sum(sum(Po))

ax2 = fig.add_subplot(122, projection='3d')
X = np.arange(20,40,0.5)
Y = np.arange(40,60,0.5)
X, Y = np.meshgrid(X, Y)
surf = ax2.plot_surface(X, Y, Po,rstride=2, cstride = 2, cmap=cm.coolwarm, antialiased=False )
ax2.set_zlim(0, 0.015)
plt.draw()

raw_input("Press Enter to continue...")

#interative bayes

[a,b] = np.unravel_index(Po.argmax(), Po.shape)

sest = np.array([Sa[a], Sb[b]])

K = np.array([[4,0],[0,4]])
for i in range(0,x[0:1].size):
    Pr = Po
    m = 0*Pr
    for j1 in range(40):
        for j2 in range(40):
            np.linalg.det(K)
            me = np.array([Sa[j1], Sb[j2]])
            z1= 1/np.sqrt((2*3.14)**2*np.linalg.det(K))
            z2 = np.exp(-np.dot(np.dot(np.transpose(x[:,i]-me),np.linalg.inv(K)),(x[:,i]-me)/2))
            m[j1,j2] = z1 * z2
            m[j1,j2] = m[j1,j2] * Pr[j1,j2]
    Po = m/sum(sum(m))
    ax2.clear()
    surf = ax2.plot_surface(X, Y, Po,rstride=2, cstride = 2, cmap=cm.coolwarm, antialiased=False )
    [a,b] = np.unravel_index(Po.argmax(), Po.shape)
    sest = np.array([Sa[a], Sb[b]])
    ax.clear()
    ax.axis([0,10, 0,10 ])
    ax.scatter(x[0,:i],x[1,:i] , c = 'y', marker  = 'o')
    ax.scatter(Sa[a], Sb[b], c = 'b', marker = 'o')
    ax.scatter(s[0], s[1], c = 'r', marker = 'o')
    plt.draw()
print sest

