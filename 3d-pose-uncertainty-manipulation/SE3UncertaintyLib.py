import numpy as np
import scipy.linalg
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
  return np.dot(covop1(A),covop1(B)) + covop1(np.dot(B,A))

def tran2vec(T):
  """
  Compute the matrix log of the transformation matrix T 4x4.
  return a 6x1 vector in tangent coordinates computed from T.
  Convert from T to xi
  """
  C = T[:3,:3]
  r = T[:3,3]
  
  phi = rot2vec(C)
  invJ = vec2jacInv(phi)
  
  rho = np.dot(invJ,r)
  return np.hstack([rho,phi])

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
    #~ # If the angle (nr) is small, fall back on the series representation.
    #~ # In my experience this is very accurate for small phi
    C = vec2rotSeries(phi,10)
    #~ #print 'vec2rot:  used series method'
  else:
    R = hat(phi)
    C = np.eye(3) + np.sin(nr)/nr*R + (1-np.cos(nr))/(nr*nr)*np.dot(R,R)
  return C

def vec2rotSeries(phi, N):
  """"
  Build a rotation matrix using the exponential map series with N elements in the series
  
  phi: 3x1 vector
  N:   number of terms to include in the series
  
  output: 
  C: 3x3 rotation matrix
  """
  C = np.eye(3)
  xM = np.eye(3)
  cmPhi = hat(phi)
  for n in range(N):
    xM = np.dot(xM, cmPhi)/(n+1)
    C = C + xM
  # Project the resulting rotation matrix back onto SO(3)
  C = np.dot(C,np.linalg.inv(scipy.linalg.sqrtm(np.dot(C.T,C))))
  return C

def cot(x):
  return 1./np.tan(x)
  
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
      invJSO3 = 0.5*nr*cot(nr*0.5)*np.eye(3) + (1- 0.5*nr*cot(0.5*nr))*axis[np.newaxis]*axis[np.newaxis].T- 0.5*nr*hat(axis)
    return invJSO3
  elif vec.shape[0] == 6: # invJacobian of SE3
    rho = vec[:3]
    phi = vec[3:]
    
    nr = np.linalg.norm(phi)
    if nr < tiny:
      # If the angle is small, fall back on the series representation
      invJSO3 = vec2jacInvSeries(phi,10)
    else:
      invJSO3 = vec2jacInv(phi)
    Q = vec2Q(vec)
    invJSE3 = np.zeros((6,6))
    invJSE3[:3,:3] = invJSO3
    invJSE3[:3,3:] = -np.dot(np.dot(invJSO3,Q), invJSO3)
    invJSE3[3:,3:] = invJSO3
    return invJSE3
  else:
    raise ValueError("Invalid input vector length\n")

def vec2jacInvSeries(vec,N):
  """
  Construction of the 3x3 J^-1 matrix or 6x6 J^-1 matrix. Series representation
  
  input: 
  vec: 3x1 vector or 6x1 vector
  N:   number of terms to include in the series
  
  output: 
  invJ: 3x3 inv(J) matrix or 6x6 inv(J) matrix
  """
  if vec.shape[0] == 3: # invJacobian of SO3
    invJSO3 = np.eye(3)
    pxn = np.eye(3)
    px = hat(vec)
    for n in range(N):
      pxn = np.dot(pxn,px)/(n+1)
      invJSO3 = invJSO3 + bernoullinumber(n+1)*pxn
    return invJSO3
  elif vec.shape[0] == 6: # invJacobian of SE3
    invJSE3 =np.eye(6)
    pxn = np.eye(6)
    px = curlyhat(vec)
    for n in range(N):
      pxn = np.dot(pxn,px)/(n+1)
      invJSE3 = invJSE3 + bernoullinumber(n+1)*pxn
    return invJSE3
  else:
    raise ValueError("Invalid input vector length\n")

def bernoullinumber(n):
    from fractions import Fraction as Fr
    if n == 1: return -0.5
    A = [0] * (n+1)
    for m in range(n+1):
        A[m] = Fr(1, m+1)
        for j in range(m, 0, -1):
          A[j-1] = j*(A[j-1] - A[j])
    return A[0].numerator*1./A[0].denominator # (which is Bn)

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
      snr = np.sin(nr)
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
      Q = vec2Q(vec)
      JSE3 = np.zeros((6,6))
      JSE3[:3,:3] = JSO3
      JSE3[:3,3:] = Q
      JSE3[3:,3:] = JSO3
    return JSE3
  else:
    raise ValueError("Invalid input vector length\n")

def vec2jacSeries(vec,N):
  """ 
  Construction of the J matrix from Taylor series
  
  input: 
  phi: a 3x1 vector for SO3
       a 6x1 vector for SE3
  N:   number of terms to include in the series
  
  output:
  J: the 3x3 J matrix for SO3
     the 6x6 J matrix for SE3
  """
  
  if vec.shape[0] == 3: # Jacobian of SO3
    JSO3 = np.eye(3)
    pxn = np.eye(3)
    px = hat(vec)
    for n in range(N):
      pxn = np.dot(pxn,px)/(n+2)
      JSO3 = JSO3 + pxn
    return JSO3
  elif vec.shape[0] == 6: # Jacobian of SE3
    JSE3 = np.eye(6)
    pxn = np.eye(6)
    px = curlyhat(vec)
    for n in range(N):
      pxn = np.dot(pxn,px)/(n+2)
      JSE3 = JSE3 + pxn
    return JSE3
  else:
    raise ValueError("Invalid input vector length\n")
  return

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
  if nr == 0:
    nr = 1e-12
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

def vec2tranSeries(p, N):
  """
  Build a transformation matrix using the exponential map series with N elements in the series
  
  p: 6x1 vector
  N:   number of terms to include in the series
  
  output:
  T: 4x4 transformation matrix
  """
  T = np.eye(4)
  xM = np.eye(4)
  bpP = hat(p)
  for n in range(N):
    xM = np.dot(xM, bpP/(n+1))
    T = T + xM
  return T
  
def propagation(T1, sigma1, T2, sigma2, method = 2):
  """
  Find the total uncertainty in a compound spatial relation (Compounding two uncertain transformations)
  T1:     4x4 mean of left transformation 
  sigma1: 6x6 covariance of left transformation
  T2:     4x4 mean of right transformation
  sigma2: 6x6 covariance of right transformations
  method: integer indicating method to be used to perform compounding
          (1=second-order, 2=fourth-order)
  output:
  T:      4x4 mean of compounded transformation 
  sigma:  6x6 covariance of compounded transformation
  """
  # Compound the means
  T = np.dot(T1,T2)
  
  # Compute Adjoint of transformation T1
  AdT1 = tranAd(T1)
  sigma2prime = np.dot(np.dot(AdT1,sigma2),AdT1)
  if method == 1:
    # Second-order method
    sigma = sigma1 + sigma2prime
    
  elif method == 2:
    # Fourth-order method
    sigma1rr = sigma1[:3,:3]
    sigma1rp = sigma1[:3,3:]
    sigma1pp = sigma1[3:,3:]
    
    sigma2rr = sigma2prime[:3,:3]
    sigma2rp = sigma2prime[:3,3:]
    sigma2pp = sigma2prime[3:,3:]
    
    A1 = np.zeros((6,6))
    A1[:3,:3] = covop1(sigma1pp)
    A1[:3,3:] = covop1(sigma1rp + sigma1rp.T)
    A1[3:,3:] = covop1(sigma1pp)
    
    A2 = np.zeros((6,6))
    A2[:3,:3] = covop1(sigma2pp)
    A2[:3,3:] = covop1(sigma2rp + sigma2rp.T)
    A2[3:,3:] = covop1(sigma2pp)

    Brr = covop2(sigma1pp,sigma2rr) + covop2(sigma1rp.T,sigma2rp) + covop2(sigma1rp,sigma2rp.T) + covop2(sigma1rr,sigma2pp)
    Brp = covop2(sigma1pp,sigma2rp.T) + covop2(sigma1rp.T,sigma2pp)
    Bpp = covop2(sigma1pp, sigma2pp)
    
    B = np.zeros((6,6))
    B[:3,:3] = Brr
    B[:3,3:] = Brp
    B[3:,:3] = Brp.T
    B[3:,3:] = Bpp
    
    sigma = sigma1 + sigma2prime + 1/12.*(np.dot(A1,sigma2prime)+np.dot(sigma2prime,A1.T) + np.dot(sigma1,A2) + np.dot(sigma1,A2.T)) + B/4.
    
  return T, sigma

def fusion(Tlist, sigmalist, N = 0):
  """
  Find the total uncertainty in a compound spatial relation (Compounding two uncertain transformations)
  Tlist:     a list of 4x4 transformations
  sigmalist: a list of corresponding 6x6 covariance matrices
  N:         N == 0 (default):JacInv is computed analytically using eq. 100
             N != 0 : JacInv is computed using eq. 103, using N first terms in the eq.
  
  output:
  T:      4x4 mean of fused transformation 
  sigma:  6x6 covariance of fused transformation
  """
  assert len(Tlist) == len(sigmalist), "Invalid data list length\n"
  kmax = len(Tlist)
  
  T = Tlist[0]
  Vprv = 0
  for i in range(30): # Gauss-Newton iterations
    LHS = np.zeros(6)
    RHS = np.zeros(6)
    for k in range(kmax):
      xik = tran2vec(np.dot(T,np.linalg.inv(Tlist[k])))
      if N ==0:
        invJ = vec2jacInv(xik)
      else:
        invJ = vec2jacInvSeries(xik, N)
      invJtS = np.dot(invJ.T, np.linalg.inv(sigmalist[k]))
      LHS = LHS + np.dot(invJtS,invJ)
      RHS = RHS + np.dot(invJtS, xik)
    
    xi = -np.linalg.solve(LHS,RHS)
    print "xi", xi
    T = np.dot(vec2tran(xi),T)
    print "T", T
    sigma = np.linalg.inv(LHS)
    
    # How low did the objective function get?
    V = 0
    for k in range(kmax):
      xik = tran2vec(np.dot(T,np.linalg.inv(Tlist[k])))
      V = V + np.dot(np.dot(xik.T,np.linalg.inv(sigmalist[k])),xik)/2

    if abs(V - Vprv) < 1e-10:
      return T, sigma 
    Vprv = V
  return T, sigma
      
def visualize(Tlist,sigmalist, nsamples = 100):
  import matplotlib.cm as cm
  fig = plt.figure()
  ax = fig.add_subplot(111, projection='3d')
  cholsigmalist = []
  colors = iter(cm.rainbow(np.linspace(0, 1, len(Tlist))))
  for i in range(len(sigmalist)):
    color = next(colors)
    cholsigma = np.linalg.cholesky(sigmalist[i]).T
    Tsample = []
    for k in range(nsamples):
      vecsample = np.dot(cholsigma,np.random.randn(6,1))
      vecsample.resize(6)
      Tsample = np.dot(vec2tran(vecsample), Tlist[i])
      ax.scatter(Tsample[0,3],Tsample[1,3],Tsample[2,3], c = color)

  ax.set_autoscaley_on(False)
  ax.set_xlim([-0.5, 0.5])
  ax.set_ylim([-0.5, 0.5])
  ax.set_zlim([-0.5, 0.5])
  ax.set_xlabel('X Label')
  ax.set_ylabel('Y Label')
  ax.set_zlabel('Z Label')
  plt.show(False)
  return True

