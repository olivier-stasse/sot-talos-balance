#!/usr/bin/python
# -*- coding: utf-8 -*-1
from sot_talos_balance.create_entities_utils import create_joint_trajectory_generator
from sot_talos_balance.create_entities_utils import create_device_filters
from dynamic_graph import plug
from time import sleep
import sot_talos_balance.talos_config as conf
import os

def main(robot):
    dt = robot.timeStep;
	NJ = robot.dimension-7
	
	#create entities
    robot.traj_gen        = create_joint_trajectory_generator(dt);
    robot.filters         = create_device_filters(dt,NJ);
    robot.imu_filter      = create_imu_filter(robot, dt);
    robot.base_estimator  = create_base_estimator(robot, dt, conf);

    robot.device.control.value = 32*[0.0];
    
    #route signals
    plug(robot.traj_gen.dx,           robot.device.control);
    plug(robot.device.forceRLEG,      robot.filters.ft_RF_filter.x);
    plug(robot.device.forceLLEG,      robot.filters.ft_LF_filter.x);
    plug(robot.device.accelerometer,  robot.filters.acc_filter.x);
    plug(robot.device.gyrometer,      robot.filters.gyro_filter.x);
    plug(robot.device.state,          robot.filters.joints_kin.x); # device.state, device.joint_angles or device.motor_angles ?

    sleep(1.0);
    
    #start simulation
    os.system('rosservice call /start_dynamic_graph');
    sleep(1.0);
    robot.traj_gen.move(31,-1.0,1.0);
    sleep(1.1);
    robot.traj_gen.startSinusoid(31,3.0,2.0);
    sleep(10.0);
    robot.traj_gen.stop(31);
