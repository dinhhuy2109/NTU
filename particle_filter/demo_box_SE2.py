import numpy as np
from openravepy import *
from ParticleFilter.ParticleFilterLib import *

env = Environment()
env.SetViewer('qtcoin')

with env:
    body = RaveCreateKinBody(env,'')
    body.SetName('testbody')
    body.InitFromBoxes(numpy.array([[0,0,0.03,0.01,0.02,0.03]]),True) # set geometry as one box of extents 0.01, 0.02, 0.03

# randomly generate N particle in this uncertain area:
# x=[-0.01, 0.01]
# y=[-0.01, 0.01]
# theta = [-pi/180,pi/3]
N = 3000
bounds = [[-1e-2,1e-2],
          [-1e-2,1e-2],
          [-np.pi/10,np.pi/10]]

particles = generateParticles(N,bounds)
weight = np.ones(len(particles))
norm_weight = normalize(weight)

# Measurement parameters
o_p = 1e-3
o_n = 2/180.0*np.pi
p1 = [0.011,0,0.01]
n1 = [-1,0,0]
d1 = [p1,n1,o_p,o_n]

p2 = [0.001,-0.021,0.02]
n2 = [0,1,0]
d2 = [p2,n2,o_p,o_n]

p3 = [0.001,-0.0202,0.03]
n3 = [0,1,0]
d3 = [p3,n3,o_p,o_n]

measurements = [d1,d2,d3]


# From measurement data, calculate weight for each particle
h1 = env.plot3(p1,10, colors=[0, 1, 0])
h2 = env.plot3(p2,10, colors=[0, 1, 0])
h3 = env.plot3(p3,10, colors=[0, 1, 0])

weight_after_d1_d2_d3 = inferFromMeasurements(particles,weight,measurements,body=body,env=env)
weight_after_d1_d2 = inferFromMeasurements(particles,weight,[d1,d2],body=body,env=env)
weight_after_d1 = inferFromMeasurements(particles,weight,[d1],body=body,env=env)

# Visualize the process
visualizeParticles(particles,norm_weight,env=env,body=body)

visualizeParticles(particles,weight_after_d1,env=env,body=body)

visualizeParticles(particles,weight_after_d1_d2,env=env,body=body)

visualizeParticles(particles,weight_after_d1_d2_d3,env=env,body=body)
