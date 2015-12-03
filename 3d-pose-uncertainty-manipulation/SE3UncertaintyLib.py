import numpy as np
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt

def tranValidate(T):
  """
  Validate T
  @type T:    array 4x4 
  @param T:   transformation matrix
  """
  raise NotImplementedError
  
def rotValidate(C):
  raise NotImplementedError

def tranAd(T):
  """
  Compute Adjoint of 4x4 transformation matrix, return a 6x6 matrix
  """
  C = T[:3,:3]
  r = T[:3,3]
  AdT = np.zeros([6,6])
  AdT[:3,:3] = C
  AdT[:3,3:] = np.dot(hat(r),C)
  AdT[3:,3:] = C
  
  return AdT
  
def hat(vec):
  """
  hat operator - Find skew matrix from vec
  """
  if vec.shape[0] == 3: # skew from vec
    return np.array([[0,-vec[2],vec[1]],[vec[2],0,-vec[0]],[-vec[1],vec[0],0]])
  elif vec.shape[0] == 6:
    vechat = np.zeros((4,4))
    vechat[:3,:3] = hat(vec[3:])
    vechat[:3,3] = vec[:3]
    return vechat
  else:
    raise ValueError("Invalid vector length for hat operator\n")

def vectfromskew(r):
  return np.array([r[2,1],r[0,2],r[1,0]])

def curlyhat(vec):
  """
  Builds the 6x6 curly hat matrix from the 6x1 input
  input a 6x1 vector xi
  output a 6x6 curly hat matrix 
  """
  veccurlyhat = np.zeros((6,6))
  veccurlyhat[:3,:3] = hat(vec[3:])
  veccurlyhat[:3,3:] = hat(vec[:3])
  veccurlyhat[3:,3:] = hat(vec[3:])
  
  return veccurlyhat

def covop1(A):
  """ 
  Covariance operator 1 - eq. 44
  """
  return -np.trace(A)*np.eye(3) + A
  
def covop2(A,B):
  """ 
  Covariance operator 2 - eq. 45
  """
  return np.dot(covop1(A),covop1(B)) + covop1(np.dot(A,B))

def tran2vec(T):
  """
  Compute the matrix log of the transformation matrix T 4x4.
  return a 6x1 vector in tangent coordinates computed from T.
  Convert from T to xi
  """
  tranValidate(T)
  C = T[:3,:3]
  r = T[:3,3]
  
  phi = rot2vec(C)
  invJ = vec2jacInv(phi)
  
  rho = np.dot(invJ,r)
  return p.hstack([rho,phi])

def rot2vec(C):
  """
  Compute the matrix log of the rotation matrix C 3x3.
  Return a 3x1 vector (axis*angle) computed from C
  """
  #rotValidate(C)
  if(abs(np.trace(C)+1)>1e-10):
    if(np.linalg.norm(C-np.eye(3))<=1e-10):
      return np.zeros(3)
    else:
      phi = np.arccos((np.trace(C)-1)/2)
      return vectfromskew(phi/(2*np.sin(phi))*(C-C.T))
  else:
    eigval, eigvect = np.linalg.eig(C)
    for (i,val) in enumerate(eigval):
      if abs((val-1)) <= 1e-10:
        return pi*np.real(eigvect[:,i])

def vec2rot(phi):
  tiny = 1e-12
  #check for small angle
  nr = np.linalg.norm(phi)
  if nr < tiny:
    # If the angle (nr) is small, fall back on the series representation.
    # In my experience this is very accurate for small phi
    C = vec2rotSeries(phi,10)
    #print 'vec2rot:  used series method'
  else:
    R = hat(phi)
    C = np.eye(3) + np.sin(nr)/nr*R + (1-np.cos(nr))/(nr*nr)*np.dot(R,R)
  return C

def vec2jacInv(vec):
  tiny = 1e-12
  if vec.shape[0] == 3: # invJacobian of SO3
    phi = vec
    nr = np.linalg.norm(phi)
    if nr < tiny:
      # If the angle is small, fall back on the series representation
      invJSO3 = vec2jacInvSeries(phi,10)
    else:
      axis = phi/nr
      invJSO3 = 0.5*nr*np.cot(nr*0.5)*np.eye(3) + (1- 0.5*nr)*np.cot(0.5*nr)*axis[np.newaxis]*axis[np.newaxis].T- 0.5*nr*hat(axis)
    return invJSO3
  elif vec.shape[0] == 6: # invJacobian of SE3
    rho = vec[:3]
    phi = vec[3:]
    
    nr = np.linalg.norm(phi)
    if nr < tiny:
      # If the angle is small, fall back on the series representation
      invJSE3 = vec2jacInvSeries(phi,10):
    else:
      invJSO3 = vec2jacInv(phi)
      Q = vec2Q(vec)
      invJSE3 = np.zeros((6,6))
      invJSE3[:3,:3] = invJSO3
      invJSE3[:3,3:] = -np.dot(np.dot(invJSO3,Q), invJSO3)
      invJSE3[3:,3:] = invJSO3
    return invJSE3

def vec2jac(vec):
  tiny = 1e-12
  if vec.shape[0] == 3: # Jacobian of SO3
    phi = vec
    nr = np.linalg.norm(phi)
    if nr < tiny:
      # If the angle is small, fall back on the series representation
      JSO3 = vec2jacSeries(phi,10)
    else:
      axis = phi/nr
      cnr = np.cos(nr)
      nr = np.sin(nr)
      JSO3 = (snr/nr)*np.eye(3) + (1-snr/nr)*axis[np.newaxis]*axis[np.newaxis].T + ((1-cnr)/nr)*hat(axis)
    return JSO3
  elif vec.shape[0] == 6: # Jacobian of SE3
    rho = vec[:3]
    phi = vec[3:]
    
    nr = np.linalg.norm(phi)
    if nr < tiny:
      #If the angle is small, fall back on the series representation
      JSE3 = vec2jacSeries(phi,10);
    else:
      JSO3 = vec2jac(phi)
      Q = vec2Q(veC)
      JSE3 = np.zeros((6,6))
      JSE3[:3,:3] = JSO3
      JSE3[:3,3:] = Q
      JSE3[3:,3:] = JSO3
    return JSE3

def vec2Q(vec):
  """
  input:
  vec: a 6x1 vector 
  output:
  Q: the 3x3 Q matrix
  """
  rho = vec[:3]
  phi = vec[3:]
  
  nr = np.linalg.norm(phi)
  nr2 = nr*nr
  nr3 = nr2*nr
  nr4 = nr3*nr
  nr5 = nr4*nr
  
  cnr = np.cos(nr)
  snr = np.sin(nr)
  
  rx = hat(rho)
  px = hat(phi)
  
  t1 = 0.5*rx
  t2 = ((nr-snr)/nr3)*(np.dot(px,rx) + np.dot(rx,px) + np.dot(np.dot(px,rx),px))
  m3 = (1-0.5*nr2 - cnr)/nr4
  t3 = -m3*(np.dot(np.dot(px,px),rx) +np.dot(np.dot(rx,px),px) -3*np.dot(np.dot(px,rx),px))
  m4 = 0.5*(m3 - 3*(nr - snr -nr3/6)/nr5)
  t4 = -m4*(np.dot(px,np.dot(np.dot(rx,px),px)) + np.dot(px,np.dot(np.dot(px,rx),px)))
  Q = t1 + t2 + t3 + t4
  return Q

def vec2tran(vec):
  """
  Build a transformation matrix using the exponential map, closed form
  vec: 6x1 vector
  output:  T: 4x4 transformation matrix
  """
  rho = vec[:3]
  phi = vec[3:]
  
  C = vec2rot(phi)
  JSO3 = vec2jac(phi)
  
  T = np.eye(4)
  T[:3,:3] = C
  T[:3,3] = np.dot(JSO3,rho)
  return T
