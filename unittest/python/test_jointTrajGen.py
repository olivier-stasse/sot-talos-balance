#!/usr/bin/python
# -*- coding: utf-8 -*-1
from sot_talos_balance.create_entities_utils import create_joint_trajectory_generator
from sot_talos_balance.create_entities_utils import create_joint_position_controller
from dynamic_graph import plug



# Waiting for services
try:
    dt = robot.timeStep;

    robot.traj_gen = create_joint_trajectory_generator(robot.device,dt)
    robot.joint_pos_controller =create_joint_position_controller(robot,dt);
    plug(robot.traj_gen.q,                       robot.joint_pos_controller.qDes);
    plug(robot.traj_gen.dq,                       robot.joint_pos_controller.dqDes);
    plug(robot.joint_pos_controller.dqRef,     robot.device.control);
    robot.joint_pos_controller.init(tuple([1.0]*32));


    sleep(1.0)
    os.system('rosservice call /start_dynamic_graph');
