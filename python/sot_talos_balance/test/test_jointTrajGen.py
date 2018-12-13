#!/usr/bin/python
# -*- coding: utf-8 -*-1
from sot_talos_balance.create_entities_utils import create_joint_trajectory_generator
from dynamic_graph import plug
from time import sleep
import os

def main(robot):
    dt = robot.timeStep;

    robot.traj_gen = create_joint_trajectory_generator(dt);
    robot.device.control.value = 32*[0.0];
    plug(robot.traj_gen.dx,    robot.device.control);

    sleep(1.0);
    os.system('rosservice call /start_dynamic_graph');
    sleep(1.0);
    robot.traj_gen.move(31,-1.0,1.0);
    sleep(1.1);
    robot.traj_gen.startSinusoid(31,3.0,2.0);
    sleep(10.0);
    robot.traj_gen.stop(31);
