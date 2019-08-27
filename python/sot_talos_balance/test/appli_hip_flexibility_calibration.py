import sot_talos_balance.talos.parameter_server_conf as paramServerConfig
import sot_talos_balance.talos.control_manager_conf as controlManagerConfig
import sot_talos_balance.talos.base_estimator_conf as baseEstimatorConf
import sot_talos_balance.talos.ft_wrist_calibration_conf as forceConf
from sot_talos_balance.create_entities_utils import *
from dynamic_graph import plug
from dynamic_graph.ros import RosSubscribe, RosPublish
import numpy as np
import math

robot.timeStep = robot.device.getTimeStep()
dt = robot.timeStep

# --- EXPERIMENTAL SET UP ------------------------------------------------------

device = 'simu'
robot.device.setControlInputType("noInteg")
N_JOINTS = 32
robot_name = "robot"


# --- SET INITIAL CONFIGURATION ------------------------------------------------

robot.halfSitting = [0., 0., 1.018213, 0., 0., 0.]  # Base
robot.halfSitting += [0., 0., -0.411354, 0.859395, -0.448041, -0.001708]  # Left Leg
robot.halfSitting += [0., 0., -0.411354, 0.859395, -0.448041, -0.001708]  # Right Leg
robot.halfSitting += [0.0,  0.006761]  # Chest
robot.halfSitting += [0.25847, 0.173046, -0.0002, -0.525366, 0., 0., 0.1, -0.005]  # Left Arm
robot.halfSitting += [-0.25847, -0.173046, 0.0002, -0.525366, 0., 0., 0.1, -0.005]  # Right Arm
robot.halfSitting += [0.,  0.]  # Head
robot.device.set(robot.halfSitting)
robot.device.control.value = robot.halfSitting

# --- CREATE ENTITIES ----------------------------------------------------------

robot.param_server = create_parameter_server(paramServerConfig, robot.timeStep)

# robot.controlManager = create_ctrl_manager(controlManagerConfig, robot.timeStep)
# robot.controlManager.addCtrlMode('pos')
# robot.controlManager.setCtrlMode('all', 'pos')

# # --- Position Controller ----------------------------------------------------------------

# robot.traj_gen = create_joint_trajectory_generator(dt, robot)


# --- HIP TASK ----------------------------------------------------------------

robot.hipCalib = create_hip_flexibility_calibration(robot, robot_name)


# --- PLUG CONTROL ----------------------------------------------------------------
# Plug control to device through control manager
# plug(robot.hipCalib.q_cmd, robot.controlManager.ctrl_pos)
# plug(robot.controlManager.u_safe, robot.device.control)
# plug(robot.hipCalib.q_cmd, robot.device.control)

# # --- ROS PUBLISHER ----------------------------------------------------------

robot.publisher = create_rospublish(robot, 'robot_publisher')
create_topic(robot.publisher, robot.hipCalib, 'theta_diff', robot=robot, data_type='vector')
create_topic(robot.publisher, robot.hipCalib, 'q_cmd', robot=robot, data_type='vector')
create_topic(robot.publisher, robot.hipCalib, 'q_des', robot=robot, data_type='vector')
create_topic(robot.publisher, robot.hipCalib, 'tau', robot=robot, data_type='vector')

# # --- ROS SUBSCRIBER
robot.subscriber = RosSubscribe("hip_calib_subscriber")
robot.subscriber.add("vector", "theta_diff", "/sot/hip_calib/theta_diff")
robot.subscriber.add("vector", "q_cmd", "/sot/hip_calib/q_cmd")
robot.subscriber.add("vector", "q_des", "/sot/hip_calib/q_des")
robot.subscriber.add("vector", "tau", "/sot/hip_calib/tau")

