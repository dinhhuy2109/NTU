from numpy import *

import lie

import TOPP
from TOPP import TOPPpy
from TOPP import TOPPbindings
from TOPP import Trajectory
from pylab import *

with open ("lie.traj", "r") as myfile0:
    trajstr=myfile0.read()

with open ("lie.constraints", "r") as myfile1:
    constraintsstr=myfile1.read()

vmax = ones(3)
discrtimestep= 1e-2

abc = TOPPbindings.RunComputeSO3Constraints(trajstr,constraintsstr)
a,b,c = lie.Extractabc(abc)
# a,b,c = lie.ComputeSO3Constraints(traj, taumax, discrtimestep) #This is the implementation of computing SO3Constraints in Python
traj = Trajectory.PiecewisePolynomialTrajectory.FromString(trajstr)
topp_inst = TOPP.QuadraticConstraints(traj, discrtimestep, vmax, list(a), list(b), list(c))

x = topp_inst.solver

ret = x.RunComputeProfiles(0,0)
if ret == 1:
    x.ReparameterizeTrajectory()
    x.WriteResultTrajectory()

traj1 = Trajectory.PiecewisePolynomialTrajectory.FromString(x.restrajectorystring)
