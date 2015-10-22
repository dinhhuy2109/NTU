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
delta0 = 5.
zoom = 2**(-1./dim)
ptcl0 = ParticleFilterLib.Particle([0,0,0],[0,0,0]) 
V0 = ParticleFilterLib.Region(ptcl0, delta0)

M = 6 # No. of particles per delta-neighbohood

delta_desired = 1. # Terminal value of delta
N = np.log2((delta/delta_desired)**dim)

delta_prv = delta0
V_prv = V0
for n in range(N):
    delta = zoom*delta_prv
    tau = (delta/delta_desired)**2
    # Sample new set of particles based on from previous region and M
    list_particles = ParticleFilterLib.EvenDensityCover(V_prv,M)
    print "No. of particles of the ", n+1, " run: ", len(list_particles), "particles"
    # Compute normalized weights
    
    # Prune based on weights

    # Create a new region from the set of particle left after pruning


    
