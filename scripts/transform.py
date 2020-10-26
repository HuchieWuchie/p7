import numpy as np
from math import cos, sin, pi


def rotx(ang):
    rotMatrix = np.array([
                        [1, 0, 0],
                        [0, cos(ang), -sin(ang)],
                        [0, sin(ang), cos(ang)]])
    return rotMatrix


def roty(ang):
    rotMatrix = np.array([
                        [cos(ang), 0, sin(ang)],
                        [0, 1, 0],
                        [-sin(ang), 0, cos(ang)]])
    return rotMatrix


def rotz(ang):
    rotMatrix = np.array([
                        [cos(ang), -sin(ang), 0],
                        [sin(ang), cos(ang), 0],
                        [0, 0, 1]])
    return rotMatrix

def rotxyz(xAng,yAng,zAng):
    return np.matmul(np.matmul(rotx(xAng), roty(yAng)), rotz(zAng))


def transformRotation(xAng,yAng,zAng):
    return np.block( [  [rotxyz(xAng,yAng,zAng), np.array([[0],[0],[0]])], [np.array([0,0,0,1])]  ]  )

def transformTranslation(x,y,z):
    return np.block([ [np.eye(3,3), np.array([[x],[y],[z]]) ],  [np.array([0,0,0,1])]  ]   )

def transformFull(xAng,yAng,zAng,x,y,z):
    return np.matmul(transformRotation(xAng,yAng,zAng), transformTranslation(x,y,z))

def transformInverse(mat):
    return np.linalg.inv(mat)

def denavitToTransform(alpha, a, d, theta):
    """ Using notation from 'introduction to robotics, mechanics and control - john j. craig' """
    transform = np.array([
                        [cos(theta), -sin(theta), 0, a],
                        [sin(theta)*cos(alpha), cos(theta)*cos(alpha), -sin(alpha), -sin(alpha)*d],
                        [sin(theta)*sin(alpha), cos(theta)*sin(alpha), cos(alpha), cos(alpha)*d],
                        [0, 0, 0, 1]])
    return transform
