# -*- coding: utf-8 -*-1
"""
2019, LAAS/CNRS
@author: Andrea Del Prete, Gabriele Buondonno
This module contains utilities for the SOT
"""
from __future__ import print_function

import os
from time import sleep

import numpy as np


class Bunch:
    def __init__(self, **kwds):
        self.__dict__.update(kwds)

    def __str__(self, prefix=""):
        res = ""
        for (key, value) in self.__dict__.iteritems():
            if (isinstance(value, np.ndarray) and len(value.shape) == 2 and value.shape[0] > value.shape[1]):
                res += prefix + " - " + key + ": " + str(value.T) + "\n"
            elif (isinstance(value, Bunch)):
                res += prefix + " - " + key + ":\n" + value.__str__(prefix + "    ") + "\n"
            else:
                res += prefix + " - " + key + ": " + str(value) + "\n"
        return res[:-1]


def start_sot():
    os.system('rosservice call /start_dynamic_graph')


def stop_sot():
    os.system('rosservice call /stop_dynamic_graph')


def smoothly_set_signal_to_zero(sig):
    value_type = type(sig.value)
    v = np.array(sig.value)
    for i in range(40):
        v = np.array(0.95 * v)
        sig.value = value_type(v)
        sleep(1)
    print('Setting signal to zero')
    v[:] = 0.0
    sig.value = value_type(v)


def smoothly_set_signal(sig, final_value, duration=5.0, steps=500, prints=10):
    value_type = type(sig.value)
    v = np.array(sig.value)
    vf = np.array(final_value)
    for i in range(steps + 1):
        alpha = 1.0 * i / steps
        sig.value = value_type(np.array(vf * alpha + (1 - alpha) * v))
        sleep(1.0 * duration / steps)
    print('Signal set')
    sig.value = value_type(vf)


def monitor_tracking_error(sig, sigRef, dt, time):
    N = int(time / dt)
    err = np.zeros((N, 6))
    for i in range(N):
        err[i, :] = np.array(sig.value) - np.array(sigRef.value)
        sleep(dt)
    for i in range(6):
        print('Max tracking error for axis %d:         %.2f' % (i, np.max(np.abs(err[:, i]))))
        print('Mean square tracking error for axis %d: %.2f' % (i, np.linalg.norm(err[:, i]) / N))


def go_to_position(traj_gen,q,T=10.0):
    #put the robot in position q
    # RLEG TO 0 **********************
    traj_gen.move(0,q[0],T) #0
    traj_gen.move(1,q[1],T) #1
    traj_gen.move(2,q[2],T) #2
    traj_gen.move(3,q[3],T) #3
    traj_gen.move(4,q[4],T) #4
    traj_gen.move(5,q[5],T) #5

    # LLEG TO 0 ************
    traj_gen.move(6,q[6],T) #6
    traj_gen.move(7,q[7],T) #7
    traj_gen.move(8,q[8],T) #8
    traj_gen.move(9,q[9],T) #9
    traj_gen.move(10,q[10],T) #10
    traj_gen.move(11,q[11],T) #11

    # TORSO TO 0
    traj_gen.move(12,q[12],T) #12
    traj_gen.move(13,q[13],T) #13

    # RARM TO 0 ************
    traj_gen.move(14,q[14],T) #14
    traj_gen.move(15,q[15],T) #15
    traj_gen.move(16,q[16],T) #16
    traj_gen.move(17,q[17],T) #17
    traj_gen.move(18,q[18],T) #28
    traj_gen.move(19,q[19],T) #19
    traj_gen.move(20,q[20],T) #20
    traj_gen.move(21,q[21],T) #21

    # LARM TO 0 ************
    traj_gen.move(22,q[22],T) #22
    traj_gen.move(23,q[23],T) #23
    traj_gen.move(24,q[24],T) #24
    traj_gen.move(25,q[25],T) #25
    traj_gen.move(26,q[26],T) #26
    traj_gen.move(27,q[27],T) #27
    traj_gen.move(28,q[28],T) #28
    traj_gen.move(29,q[29],T) #29

    # HEAD TO 0
    traj_gen.move(30,q[30],T) #30
    traj_gen.move(31,q[31],T) #31
