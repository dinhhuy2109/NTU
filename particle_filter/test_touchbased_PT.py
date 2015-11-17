from numpy import *
from openravepy import *
import ParticleFilterLib
from pylab import *
import Transformation as Tf

env = Environment()
env.SetViewer('qtcoin')
woodstick = env.ReadKinBodyXMLFile("woodstick.xml")
#env.AddKinBody(woodstick,True)
ion()

# Measurements
o_p = 5e-3
o_n = 5/180.0*np.pi

p1 = [-0.001,0.039,-0.055] # side near y
n1 = [1,0,0.1]
d1 = [p1,n1,o_p,o_n]

p3 = [0.02,0.009,-0.015] # side near y
n3 = [-1,0.002,0]
d3 = [p3,n3,o_p,o_n]

p2 = [0.015,0.0152, 0.001] # top
n2 = [0,0.02,-1]
d2 = [p2,n2,o_p,o_n]

p4 = [0.005,0.043, 0.00] # top
n4 = [0,0.0,-1]
d4 = [p4,n4,o_p,o_n]

p5 = [0.012,0.051,-0.03] # side near x
n5 = [0.,-1,0.0]
d5 = [p5,n5,o_p,o_n]

p6 = [0.019,-0.001,-0.044] # side near x
n6 = [0.001,1,0.001]
d6 = [p6,n6,o_p,o_n]

D = [d3,d2]#,d1,d4,d6]

T = Tf.euler_matrix(3./180.*pi,-14/180.*pi,22./180.*pi)
T[:3,3]= array([0.012,-0.018,-0.020])
for d in D:
    d[0] = dot(T[:3,:3],d[0]) + T[:3,3]
    d[1] = dot(T[:3,:3],d[1]) + T[:3,3]

handles = []
for d in D:
    handles.append(env.plot3(d[0],0.001, colors=[0, 1, 0],drawstyle=1))
    
#~ raw_input("Press Enter to continue...")
delta0 = 20
dim = 6 # 6 DOFs
ptcl0 = ParticleFilterLib.Particle([0,0,0],[0,0,0]) 
V0 = ParticleFilterLib.Region([ptcl0], delta0)    
M = 6 # No. of particles per delta-neighbohood

delta_desired = 1# Terminal value of delta
list_particles, weights = ParticleFilterLib.ScalingSeries(V0, D, M, delta0, delta_desired,  dim, body = woodstick, env = env, visualize = True)

est = ParticleFilterLib.VisualizeParticles(list_particles, weights, env= env, body=woodstick, showestimated = True)


