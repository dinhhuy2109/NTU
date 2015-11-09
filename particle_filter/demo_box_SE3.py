from numpy import *
from openravepy import *
import ParticleFilterLib
from pylab import *
import Transformation as Tf

env = Environment()
env.SetViewer('qtcoin')
# env.Load("woodstick.xml")
# woodstick = env.GetBodies()[0]
# ion()

body = RaveCreateKinBody(env,'')
body.SetName('testbody')
body.InitFromBoxes(array([[0,0,0,0.01,0.02,0.03]]),True) # set geometry as one box of extents 0.01, 0.02, 0.03

# Measurement parameters
o_p = 2e-3
o_n = 10/180.0*np.pi

p1 = [0.01,0.015,0.025]
n1 = [-1,0,0.01]
d1 = [p1,n1,o_p,o_n]

p2 = [-0.01,-0.0102,0.02]
n2 = [1,0.01,0]
d2 = [p2,n2,o_p,o_n]

p8 = [0.011,-0.018,0.024]
n8 = [-1,0.02,0.01]
d8 = [p8,n8,o_p,o_n]


p3 = [-0.007,0.020,0.025]
n3 = [0,-1,0]
d3 = [p3,n3,o_p,o_n]

p4 = [0.008,-0.02,0.02]
n4 = [0,1,-0.04]
d4 = [p4,n4,o_p,o_n]



p5 = [-0.005,0.0142,0.03]
n5 = [0.07,0,-1]
d5 = [p5,n5,o_p,o_n]

p6 = [0.002,-0.0,0.03]
n6 = [0.06,-0.07,-1]
d6 = [p6,n6,o_p,o_n]

D = [d1,d2,d3,d4,d5,d6,d8]

T = Tf.euler_matrix(20./180.*pi,-14/180.*pi,32./180.*pi)
T[:3,3]= array([0.012,-0.018,-0.035])
for d in D:
    d[0] = dot(T[:3,:3],d[0]) + T[:3,3]
    d[1] = dot(T[:3,:3],d[1]) + T[:3,3]

# From measurement data, calculate weight for each particle
handles = []
for d in D:
    handles.append(env.plot3(d[0],0.001, colors=[0, 1, 0],drawstyle=1))
# env.AddKinBody(body)
raw_input("Press Enter to continue...")
# Create data
delta0 = 50
dim = 6 # 6 DOFs
ptcl0 = ParticleFilterLib.Particle([0,0,0],[0,0,0]) 
V0 = ParticleFilterLib.Region([ptcl0], delta0)    
M = 6 # No. of particles per delta-neighbohood

delta_desired = 1 # Terminal value of delta
list_particles, weights = ParticleFilterLib.ScalingSeries(V0, D, M, delta0, delta_desired, body = body, env = env)

est = ParticleFilterLib.VisualizeParticles(list_particles, weights, env= env, body=body, showestimated = True)

# Show the real box
box = RaveCreateKinBody(env,'')
box.SetName('box')
box.InitFromBoxes(array([[0,0,0.0,0.01,0.02,0.03]]),True)
box.SetTransform(T)
env.AddKinBody(box)

ptcl0 = ParticleFilterLib.Particle(np.dot(est,[0,0,0,1])[:3],np.dot(est,[0,0,0,1])[:3])
V0 = ParticleFilterLib.Region([ptcl0], delta0)  
delta0 = 15
delta_desired = 1
list_particles, weights = ParticleFilterLib.ScalingSeries(V0, D, M, delta0, delta_desired, body = body, env = env)
est = ParticleFilterLib.VisualizeParticles(list_particles, weights, env= env, body=body, showestimated = True)
