from openravepy import *
from numpy import *

from SO3RRT import *
import Utils
import lie

import time
import TOPP
from TOPP import TOPPpy
from TOPP import TOPPbindings
from TOPP import Trajectory
from pylab import *
import scipy.optimize
from mpl_toolkits.mplot3d import Axes3D




##########################LOAD LIETRAJ #################################
# ion()
# env = Environment()
# # This model was downloaded from http://nasa3d.arc.nasa.gov/models/printable
# env.Load("MESSENGER/messengerWithEnv.xml")
# env.SetViewer('qtcoin')
# robot = env.GetBodies()[0]

# taumax = ones(3)
# vmax = ones(3)
# inertia = eye(3)
# print "\033[93mLOADING lietraj","\033[0m"
# lietraj1 = Utils.ReadLieTrajFiles("Rlist1.txt", "trajlist1.txt")
# # for t in linspace(0, lietraj.duration, 1000): 
# #     M[:3,:3] = lietraj.EvalRotation(t)
# #     robot.SetTransform(M)
# #     isincollision = (env.CheckCollision(robot, CollisionReport()))
# #     if (isincollision):
# #         print "in collision", " ", t, "/" , lietraj4.duration
# #     time.sleep(0.01)
# print "\033[93mDONE","\033[0m"

# print "\033[93mRunning SHORTCUTTING", "\033[0m"

# lietraj2 = Utils.Shortcut(robot, taumax, vmax, lietraj1, 200, -1, 0, -1, inertia)

# print "\033[93mDone", "\033[0m"

# print "\033[1;94mFinal trajectory duration: ", lietraj2.duration, " sec.\033[0m"
# lietraj2.Plot(0.01,0,vmax,taumax,taumax,inertia)


# lietraj2 = Utils.ReadLieTrajFiles("Rlist2good1.txt", "trajlist2good1.txt")
# taumax = ones(3)
# vmax = ones(3)
# inertia = eye(3)
# lietraj2.Plot(0.01,0,vmax,taumax,taumax,inertia)

#######################SE3##################33

ion()



env = Environment()
# This model was downloaded from http://nasa3d.arc.nasa.gov/models/printable
env.Load("MESSENGER/messengerWithEnvSE3.xml")
env.SetViewer('qtcoin')

robot = env.GetBodies()[0]

taumax = ones(3)
vmax = ones(6)
fmax = ones(3)
se3traj1, Rlist = Utils.ReadSE3TrajFiles("se3Rlist2.txt", "se3trajlist2.txt")

se3traj2, Rlist2 = Utils.SE3Shortcut(robot, taumax, fmax, vmax, se3traj1, Rlist, 200)
print "\033[1;94mFinal trajectory duration: ", se3traj2.duration, " sec.\033[0m"
#Utils.PlotSE3(se3traj2, Rlist2, 0.01, 0,vmax,taumax,taumax,fmax,np.eye(3))
