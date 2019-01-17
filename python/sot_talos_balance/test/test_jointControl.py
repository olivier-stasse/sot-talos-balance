#!/usr/bin/python
# -*- coding: utf-8 -*-1
from sot_talos_balance.create_entities_utils import create_joint_trajectory_generator
from sot_talos_balance.create_entities_utils import create_joint_controller
from dynamic_graph import plug
from time import sleep
import sys
import os

def main(robot,gain):
    N_JOINTS = 32
    Kp = N_JOINTS*[gain]
    dt = robot.timeStep

    robot.traj_gen   = create_joint_trajectory_generator(dt)
    robot.device.control.value = N_JOINTS*[0.0]

    robot.controller = create_joint_controller(Kp)

    plug(robot.traj_gen.x,    robot.controller.qDes)
    plug(robot.traj_gen.dx,   robot.controller.dqDes)

    plug(robot.device.state,    robot.controller.state)

    plug(robot.controller.dqRef,    robot.device.control)

    robot.controller.init(N_JOINTS)

    sleep(1.0)
    os.system('rosservice call /start_dynamic_graph')
    sleep(1.0)
    robot.traj_gen.move(31,-1.0,1.0)
    sleep(1.1)
    robot.traj_gen.startSinusoid(31,1.0,2.0)
    sleep(10.0)
    robot.traj_gen.stop(31)

