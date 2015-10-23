import numpy as np
from openravepy import *
from ParticleFilterLib import *

env = Environment()
env.SetViewer('qtcoin')

body = RaveCreateKinBody(env,'')
body.SetName('testbody')
body.InitFromBoxes(numpy.array([[0,0,0,0.01,0.02,0.03]]),True) # set geometry as one box of extents 0.01, 0.02, 0.03

# Create data
dim = 6. # 6 DOFs
ptcl0 = ParticleFilterLib.Particle([0,0,0],[0,0,0]) 
V0 = ParticleFilterLib.Region(ptcl0, delta0)    
