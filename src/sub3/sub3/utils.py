import socket
import numpy as np
import cv2

# -	utils_skeleton.py : 

def xyh2mat2D(vec):
    euler_rad = vec[2] * np.pi / 180
    rot = np.array([np.cos(euler_rad), -np.sin(euler_rad), np.sin(euler_rad), np.cos(euler_rad)])
    trans = vec[0:2]

    T = np.identity(3)
    T[0:2, 0:2] = rot.reshape(2, 2)
    T[0:2, 2] = trans.reshape(-1)

    # print('vec', vec)
    # print('T', T)

    return T

def mat2D2xyh(T):
    vec = np.array([0.0, 0.0, 0.0])
    t = T[:2, 2]
    rot = T[:2, :2]
    vec[0] = t[0]
    vec[1] = t[1]
    vec[2] = np.arctan2(rot[1,0], rot[0,0])*180.0/np.pi

    return vec


# radian
def limit_angular_range(rad):

    if rad > np.pi:
        rad -= 2*np.pi

    if rad < -np.pi:
        rad += 2*np.pi

    if rad > np.pi:
        rad -= 2*np.pi

    if rad < -np.pi:
        rad += 2*np.pi

    return rad
