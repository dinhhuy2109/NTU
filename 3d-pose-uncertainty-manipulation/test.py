from numpy import *
import SE3UncertaintyLib

R = array([[cos(pi/2),-sin(pi/2),0],[sin(pi/2),cos(pi/2), 0],[0,0,1]])

print SE3UncertaintyLib.vec2rot(array([0,0,1.57079]))

print SE3UncertaintyLib.rot2vec(R)
