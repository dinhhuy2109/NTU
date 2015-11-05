import numpy as np
from openravepy import *
import ParticleFilterLib

env = Environment()
env.SetViewer('qtcoin')

body = RaveCreateKinBody(env,'')
body.SetName('testbody')
body.InitFromBoxes(numpy.array([[0,0,0,0.01,0.02,0.03]]),True) # set geometry as one box of extents 0.01, 0.02, 0.03
# Measurement parameters
o_p = 3e-3
o_n = 40/180.0*np.pi
p1 = [0.011,0,0.015]
n1 = [-1,0,0]
d1 = [p1,n1,o_p,o_n]

p2 = [0.001,-0.021,0.025]
n2 = [0,1,0]
d2 = [p2,n2,o_p,o_n]

p3 = [-0.007,0.0202,0.03]
n3 = [0,-1,0]
d3 = [p3,n3,o_p,o_n]

p4 = [-0.01,-0.0102,0.02]
n4 = [1,0,0]
d4 = [p4,n4,o_p,o_n]

p5 = [-0.005,0.0142,0.035]
n5 = [0,0,-1]
d5 = [p5,n5,o_p,o_n]


D = [d1,d2,d3,d4,d5]

# From measurement data, calculate weight for each particle
handles = []
for d in D:
    handles.append(env.plot3(d[0],0.001, colors=[0, 1, 0],drawstyle=1))
# env.AddKinBody(body)
raw_input("Press Enter to continue...")
# Create data
delta0 = 6
dim = 3 # 6 DOFs
ptcl0 = ParticleFilterLib.Particle([0,0,0],[0,0,0]) 
V0 = ParticleFilterLib.Region([ptcl0], delta0)    
M = 6 # No. of particles per delta-neighbohood

delta_desired = 2 # Terminal value of delta
list_particles, weights = ParticleFilterLib.ScalingSeries(V0, D, M, delta0, delta_desired, body = body, env = env)
ParticleFilterLib.VisualizeParticles(list_particles, weights, env= env, body=body, showestimated = True)

# Show the real box
box = RaveCreateKinBody(env,'')
box.SetName('box')
box.InitFromBoxes(numpy.array([[0,0,0.005,0.01,0.02,0.03]]),True)
env.AddKinBody(box)

