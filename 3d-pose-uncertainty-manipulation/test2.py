import numpy as np
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt

def isinside(point, center, sigma):
  cholsigma = np.linalg.cholesky(sigma).T
  univariable = np.dot(np.linalg.inv(cholsigma),(point-center))
  nr = np.linalg.norm(univariable)
  if nr <= 1.0:
    return True
  else:
    return False
    
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

sigma = np.array([[1,0,0],[0,4,0],[0,0,0.1]])
cholsigma = np.linalg.cholesky(sigma).T

for k in range(500):
  uniformsample = np.random.uniform(-1,1,size = 3)
  Tsample = np.dot(cholsigma, uniformsample)
  #~ ax.scatter(Tsample[0],Tsample[1],Tsample[2], c = 'r')
  #~ normalsample = np.random.randn(3,1)
  #~ normalsample = np.random.uniform(-1,1,size =1 )*normalsample/np.linalg.norm(normalsample)
  #ax.scatter(normalsample[0],normalsample[1],normalsample[2], c = 'r')
  #~ Tsample = np.dot(cholsigma,normalsample)
  if isinside(Tsample, np.zeros(3), sigma):
    ax.scatter(Tsample[0],Tsample[1],Tsample[2], c = 'y')
  #~ else:
    #~ ax.scatter(Tsample[0],Tsample[1],Tsample[2], c = 'b')

ax.set_autoscaley_on(False)
ax.set_xlim([-0.5, 0.5])
ax.set_ylim([-0.5, 0.5])
ax.set_zlim([-0.5, 0.5])
ax.set_xlabel('X Label')
ax.set_ylabel('Y Label')
ax.set_zlabel('Z Label')
plt.show(False)
