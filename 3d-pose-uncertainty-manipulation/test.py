from numpy import *
import SE3UncertaintyLib as lib

R = array([[cos(pi/2),-sin(pi/2),0],[sin(pi/2),cos(pi/2), 0],[0,0,1]])
a = array([1,2,3])
print lib.vec2rot(array([0,0,1.57079]))

print lib.rot2vec(R)

print lib.vec2rot(a)

print lib.vec2rotSeries(a,10)

print lib.vec2jacInvSeries(array([1,2,3]),20)

print lib.vec2rotSeries(array([1,2,3]),10)
