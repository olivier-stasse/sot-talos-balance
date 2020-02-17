# flake8: noqa
import math

import numpy as np
from dynamic_graph import plug
from dynamic_graph.ros import RosPublish, RosSubscribe
from dynamic_graph.sot.core import SOT, FeaturePosture, GainAdaptive, Task
from dynamic_graph.sot.core.matrix_util import matrixToTuple
from dynamic_graph.sot.core.meta_tasks_kine import MetaTaskKine6d, MetaTaskKineCom
from dynamic_graph.tracer_real_time import TracerRealTime
from sot_talos_balance.utils.sot_utils import go_to_position

import sot_talos_balance.talos.base_estimator_conf as baseEstimatorConf
import sot_talos_balance.talos.control_manager_sim_conf as controlManagerConfig
import sot_talos_balance.talos.ft_wrist_calibration_conf as forceConf
import sot_talos_balance.talos.parameter_server_conf as paramServerConfig
from sot_talos_balance.create_entities_utils import *

robot.timeStep = robot.device.getTimeStep()

# --- EXPERIMENTAL SET UP ------------------------------------------------------

device = 'simu'
endEffector = 'rightWrist'
endEffectorWeight = forceConf.handWeight[device]
rightOC = forceConf.rightLeverArm
leftOC = forceConf.leftLeverArm

# --- SET INITIAL CONFIGURATION ------------------------------------------------

q = [0., 0., 1.018213, 0., 0., 0.]  # Base
q += [0., 0., -0.411354, 0.859395, -0.448041, -0.001708]  # Left Leg
q += [0., 0., -0.411354, 0.859395, -0.448041, -0.001708]  # Right Leg
q += [0.0, 0.006761]  # Chest
q += [0.25847, 0.173046, -0.0002, -0.525366, 0., 0., 0.1, -0.005]  # Left Arm
# q += [-0.25847, -0.173046, 0.0002, -0.525366, 0., 0., 0.1, -0.005]  # Right Arm
# q += [-0.25847, -0.0, 0.19, -1.61, 0., 0., 0.1, -0.005]             # Right Arm
q += [-0.0, -0.01, 0.00, -1.58, -0.01, 0., 0., -0.005]  # Right Arm
q += [0., 0.]  # Head

robot.positionDrill = q

# --- Initial feet and waist
robot.dynamic.createOpPoint('LF', robot.OperationalPointsMap['left-ankle'])
robot.dynamic.createOpPoint('RF', robot.OperationalPointsMap['right-ankle'])
robot.dynamic.LF.recompute(0)
robot.dynamic.RF.recompute(0)

# --- CREATE ENTITIES ----------------------------------------------------------

robot.param_server = create_parameter_server(paramServerConfig, robot.timeStep)
robot.device_filters = create_device_filters(robot, robot.timeStep)
robot.imu_filters = create_imu_filters(robot, robot.timeStep)
robot.baseEstimator = create_base_estimator(robot, robot.timeStep, baseEstimatorConf)
robot.trajGen = create_config_trajectory_generator(robot.timeStep, robot)
robot.trajGen.x.recompute(0)

# --- CoM Traj
robot.comTrajGen = create_com_trajectory_generator(robot.timeStep, robot)
robot.comTrajGen.x.recompute(0)  # trigger computation of initial value

# Get configuration vector
robot.e2q = EulerToQuat("e2q")
plug(robot.baseEstimator.q, robot.e2q.euler)

robot.forceCalibrator = create_ft_wrist_calibrator(robot, endEffectorWeight, rightOC, leftOC)
robot.controller = create_end_effector_admittance_controller(robot, endEffector, "EEAdmittance")

robot.controlManager = create_ctrl_manager(controlManagerConfig, robot.timeStep)
robot.controlManager.addCtrlMode('sot_input')
robot.controlManager.setCtrlMode('all', 'sot_input')

# --- CONTACTS ----------------------------------------------------------------
#define contactLF and contactRF
robot.contactLF = MetaTaskKine6d('contactLF', robot.dynamic, 'LF', robot.OperationalPointsMap['left-ankle'])
robot.contactLF.feature.frame('desired')
robot.contactLF.gain.setConstant(100)
robot.contactLF.keep()
locals()['contactLF'] = robot.contactLF

robot.contactRF = MetaTaskKine6d('contactRF', robot.dynamic, 'RF', robot.OperationalPointsMap['right-ankle'])
robot.contactRF.feature.frame('desired')
robot.contactRF.gain.setConstant(100)
robot.contactRF.keep()
locals()['contactRF'] = robot.contactRF

# --- COM TASK ----------------------------------------------------------------
robot.taskCom = MetaTaskKineCom(robot.dynamic)
plug(robot.comTrajGen.x, robot.taskCom.featureDes.errorIN)
robot.taskCom.task.controlGain.value = 10.
robot.taskCom.feature.selec.value = '011'


# --- HAND TASK ----------------------------------------------------------------

robot.taskRightHand = MetaTaskKine6d('rh', robot.dynamic, 'rh', 'arm_right_7_joint')
handMgrip = np.eye(4)
handMgrip[0:3, 3] = (0.1, 0, 0)
robot.taskRightHand.opmodif = matrixToTuple(handMgrip)
robot.taskRightHand.feature.frame('desired')
robot.taskRightHand.feature.selec.value = '111111'
robot.taskRightHand.task.setWithDerivative(True)
robot.taskRightHand.task.controlGain.value = 0
robot.taskRightHand.feature.position.value = np.eye(4)
robot.taskRightHand.feature.velocity.value = [0., 0., 0., 0., 0., 0.]
robot.taskRightHand.featureDes.position.value = np.eye(4)
plug(robot.controller.dq, robot.taskRightHand.featureDes.velocity)

# --- BASE TASK ----------------------------------------------------------------

robot.taskWaist = MetaTaskKine6d('taskWaist', robot.dynamic, 'WT', robot.OperationalPointsMap['waist'])
robot.taskWaist.feature.frame('desired')
robot.taskWaist.gain.setConstant(300)
robot.taskWaist.keep()
robot.taskWaist.feature.selec.value = '111111'
locals()['taskWaist'] = robot.taskWaist

# --- POSTURE TASK -------------------------------------------------------------

robot.taskPosture = Task('task_posture')
robot.taskPosture.controlGain.value = 100.0
robot.taskPosture.feature = FeaturePosture('feature_posture')

q = list(robot.dynamic.position.value)
plug(robot.dynamic.position, robot.taskPosture.feature.state)
plug(robot.trajGen.x, robot.taskPosture.feature.posture)

robot.taskPosture.feature.selectDof(6, True)
robot.taskPosture.feature.selectDof(7, True)
robot.taskPosture.feature.selectDof(8, True)
robot.taskPosture.feature.selectDof(9, True)
robot.taskPosture.feature.selectDof(10, True)
robot.taskPosture.feature.selectDof(11, True)
robot.taskPosture.feature.selectDof(12, True)
robot.taskPosture.feature.selectDof(13, True)
robot.taskPosture.feature.selectDof(14, True)
robot.taskPosture.feature.selectDof(15, True)
robot.taskPosture.feature.selectDof(16, True)
robot.taskPosture.feature.selectDof(17, True)
robot.taskPosture.feature.selectDof(18, True)
robot.taskPosture.feature.selectDof(19, True)
robot.taskPosture.feature.selectDof(20, True)
robot.taskPosture.feature.selectDof(21, True)
robot.taskPosture.feature.selectDof(22, True)
robot.taskPosture.feature.selectDof(23, True)
robot.taskPosture.feature.selectDof(24, True)
robot.taskPosture.feature.selectDof(25, True)
robot.taskPosture.feature.selectDof(26, True)
robot.taskPosture.feature.selectDof(27, True)
robot.taskPosture.feature.selectDof(28, True)
robot.taskPosture.feature.selectDof(29, True)
robot.taskPosture.feature.selectDof(30, True)
robot.taskPosture.feature.selectDof(31, True)
robot.taskPosture.feature.selectDof(32, True)
robot.taskPosture.feature.selectDof(33, True)
robot.taskPosture.feature.selectDof(34, True)
robot.taskPosture.feature.selectDof(35, True)
robot.taskPosture.feature.selectDof(36, True)
robot.taskPosture.feature.selectDof(37, True)

robot.taskPosture.add(robot.taskPosture.feature.name)

robot.sot = SOT('sot')
robot.sot.setSize(robot.dynamic.getDimension())

# Plug SOT control to device through control manager
plug(robot.sot.control, robot.controlManager.ctrl_sot_input)
plug(robot.controlManager.u_safe, robot.device.control)
# plug(robot.sot.control, robot.device.control)

# --- PUSH THE TASKS -----------------------------------------------------------

robot.sot.push(robot.taskCom.task.name)
robot.sot.push(robot.contactRF.task.name)
robot.sot.push(robot.contactLF.task.name)
robot.sot.push(robot.taskWaist.task.name)
robot.sot.push(robot.taskPosture.name)

# # --- ROS PUBLISHER ----------------------------------------------------------

robot.publisher = create_rospublish(robot, 'robot_publisher')
create_topic(robot.publisher, robot.controller, 'w_force', robot=robot, data_type='vector')
create_topic(robot.publisher, robot.controller, 'force', robot=robot, data_type='vector')
create_topic(robot.publisher, robot.controller, 'dq', robot=robot, data_type='vector')
create_topic(robot.publisher, robot.controller, 'w_dq', robot=robot, data_type='vector')
create_topic(robot.publisher, robot.controller, 'w_forceDes', robot=robot, data_type='vector')
create_topic(robot.publisher, robot.forceCalibrator, 'leftWristForceOut', robot=robot, data_type='vector')  # calibrated left wrench
create_topic(robot.publisher, robot.forceCalibrator, 'rightWristForceOut', robot=robot, data_type='vector')  # calibrated right wrench

# # --- ROS SUBSCRIBER
robot.subscriber = RosSubscribe("end_effector_subscriber")
robot.subscriber.add("vector", "w_force", "/sot/controller/w_force")
robot.subscriber.add("vector", "force", "/sot/controller/force")
robot.subscriber.add("vector", "dq", "/sot/controller/dq")
robot.subscriber.add("vector", "w_dq", "/sot/controller/w_dq")
robot.subscriber.add("vector", "w_forceDes", "/sot/controller/w_forceDes")
robot.subscriber.add("vector", "leftWristForceOut", "/sot/forceCalibrator/leftWristForceOut")
robot.subscriber.add("vector", "rightWristForceOut", "/sot/forceCalibrator/rightWristForceOut")

# --- TRACER  ------------------------------------------------------------------

robot.tracer = TracerRealTime("force_tracer")
robot.tracer.setBufferSize(80 * (2**20))
robot.tracer.open('/tmp', 'dg_', '.dat')
robot.device.after.addSignal('{0}.triger'.format(robot.tracer.name))

addTrace(robot.tracer, robot.controller, 'force')
addTrace(robot.tracer, robot.controller, 'w_force')
addTrace(robot.tracer, robot.controller, 'w_dq')
addTrace(robot.tracer, robot.controller, 'dq')

robot.tracer.start()
