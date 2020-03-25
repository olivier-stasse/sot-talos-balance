# flake8: noqa
import math

import numpy as np
from dynamic_graph import plug
from dynamic_graph.ros import RosPublish, RosSubscribe
from dynamic_graph.sot.core import SOT, FeaturePosture, GainAdaptive, Task, FeaturePose
from dynamic_graph.sot.core.matrix_util import matrixToTuple
from dynamic_graph.sot.core.meta_tasks_kine import MetaTaskKine6d, MetaTaskKineCom
from dynamic_graph.sot.core import GainAdaptive
from dynamic_graph.sot.core import OpPointModifier
from dynamic_graph.sot.core.operator import Norm_of_vector
from dynamic_graph.sot.core.operator import CompareDouble 
from dynamic_graph.sot.core.operator import Multiply_matrixTwist_vector
from dynamic_graph.sot.core.operator import HomoToTwist
from dynamic_graph.sot.core.switch import SwitchVector

from dynamic_graph.tracer_real_time import TracerRealTime
from sot_talos_balance.utils.sot_utils import go_to_position
from sot_talos_balance.admittance_controller_op_point import AdmittanceControllerOpPoint
import sot_talos_balance.talos.base_estimator_conf as baseEstimatorConf
import sot_talos_balance.talos.control_manager_sim_conf as controlManagerConfig
import sot_talos_balance.talos.ft_wrist_calibration_conf as forceConf
import sot_talos_balance.talos.parameter_server_conf as paramServerConfig
from sot_talos_balance.create_entities_utils import *
import pinocchio as pin

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
robot.dynamic.createOpPoint('WT', robot.OperationalPointsMap['waist'])
robot.dynamic.createOpPoint('World', "universe")
robot.dynamic.createOpPoint('RW_joint', "arm_right_7_joint")
robot.dynamic.createOpPoint('RW_sensor', "wrist_right_ft_link")
robot.dynamic.LF.recompute(0)
robot.dynamic.RF.recompute(0)
robot.dynamic.WT.recompute(0)
robot.dynamic.World.recompute(0)
robot.dynamic.RW_joint.recompute(0)
robot.dynamic.RW_sensor.recompute(0)

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

robot.controlManager = create_ctrl_manager(controlManagerConfig, robot.timeStep)
robot.controlManager.addCtrlMode('sot_input')
robot.controlManager.setCtrlMode('all', 'sot_input')


# --- HAND TASKS ----------------------------------------------------------------

# Hole position
jaMfa = np.zeros((4,4)) # pose of the hole frame fa wrt world ja
jaMfa[0,2] = -1
jaMfa[1,1] = 1
jaMfa[2,0] = 1
jaMfa[0:4, 3] = [0.67, -0.382, 1.125, 1] # pose of the drill when put in position + 0.1 on the x axis 

# Drill position
jbMfb = np.eye(4) # pose of the drill (foret) fb frame wrt wrist jb
jbMfb[0:3, 3] = [0.05, 0.0, -0.3]

robot.drillOpPoint = OpPointModifier('drill')
plug(robot.dynamic.RW_joint,  robot.drillOpPoint.positionIN)
plug(robot.dynamic.JRW_joint, robot.drillOpPoint.jacobianIN)
robot.drillOpPoint.setTransformation(jbMfb)
robot.drillOpPoint.position.recompute(0)
robot.drillOpPoint.jacobian.recompute(0)

# Task position control
robot.taskRightHandPos = Task('taskRightHandPos')
robot.taskRightHandPos.setWithDerivative(True)
robot.taskRightHandPos.controlGain.value = 100

featureRH_pos = FeaturePose('featureRH_pos')
featureRH_pos.selec.value = "101000"
plug(robot.dynamic.World, featureRH_pos.oMja) # pose of joint ja (world) in the world
plug(robot.dynamic.JWorld, featureRH_pos.jaJja) # jacobian of joint ja (world)
plug(robot.dynamic.RW_joint, featureRH_pos.oMjb) # pose of joint jb (wrist) in the world
plug(robot.dynamic.JRW_joint, featureRH_pos.jbJjb) # jacobian of jb (wrist)
featureRH_pos.jaMfa.value = jaMfa # pose of the frame fa (hole) wrt ja (world)
featureRH_pos.jbMfb.value = jbMfb # pose of the frame fb (drill) wrt jb (wrist)
featureRH_pos.faMfbDes.value = np.eye(4)
featureRH_pos.faNufafbDes.value = [0., 0., 0., 0., 0., 0.]

# Task admittance control
robot.taskRightHandAdm = Task('taskRightHandAdm')
robot.taskRightHandAdm.setWithDerivative(True)
robot.taskRightHandAdm.controlGain.value = 0

featureRH_adm = FeaturePose('featureRH_adm')
featureRH_adm.selec.value = "010111"
plug(robot.dynamic.World, featureRH_adm.oMja) # pose of joint ja (world) in the world
plug(robot.dynamic.JWorld, featureRH_adm.jaJja) # jacobian of joint ja (world)
plug(robot.dynamic.RW_joint, featureRH_adm.oMjb) # pose of joint jb (wrist) in the world
plug(robot.dynamic.JRW_joint, featureRH_adm.jbJjb) # jacobian of jb (wrist)
featureRH_adm.jaMfa.value = jaMfa # pose of the frame fa (hole) wrt ja (world)
featureRH_adm.jbMfb.value = jbMfb # pose of the frame fb (drill) wrt jb (wrist)
featureRH_adm.faMfbDes.value = np.eye(4)
featureRH_adm.faMfb.recompute(0)

robot.taskRightHandPos.add('featureRH_pos')
robot.taskRightHandAdm.add('featureRH_adm')


# --- ADMITTANCE CONTROLLER ------------------------------------------------------

admController = AdmittanceControllerOpPoint("DrillAdmittance")
plug(robot.forceCalibrator.rightWristForceOut, admController.force) # rightWrist
admController.Kp.value = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
admController.Kd.value = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
admController.w_forceDes.value = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
admController.dqSaturation.value = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
plug(robot.dynamic.RW_sensor, admController.sensorPose)
plug(robot.drillOpPoint.position, admController.opPose)
admController.init(robot.timeStep)
robot.controller = admController

# --- SWITCH when Force detected at the hand -----------------------------------

# Norm of the sensor force vector
norm = Norm_of_vector("force_norm")
plug(robot.forceCalibrator.rightWristForceOut, norm.sin)
# Compare norm with a threshold which returns true when force is detected
threshold = 1.5
compare = CompareDouble("compare_norm")
plug(norm.sout, compare.sin2)
compare.sin1.value = threshold

# Transformation of the velocity output of the admittance controller to the right frame
# vel_adm is the velocity desired for the drill in the drill frame
# Here we want the velocity desired for the frame fb (drill) in the frame fa (hole)
faMfbTwist = HomoToTwist("faMfbTwist") # Homogeneous Matrix to Twist
plug(featureRH_adm.faMfb, faMfbTwist.sin)
# faXfb . fbNufb = faNufb
faNufbAdm = Multiply_matrixTwist_vector("faNufbAdm")
plug(faMfbTwist.sout, faNufbAdm.sin1)
plug(robot.controller.dq, faNufbAdm.sin2)
faNufbAdm.sout.recompute(0)

# Switch which activates the control when the comparison is true
switch = SwitchVector("switch_adm")
switch.setSignalNumber(2)
plug(faNufbAdm.sout, switch.sin1)
switch.sin0.value = [0., 0., 0., 0., 0., 0.]
plug(compare.sout, switch.boolSelection)
robot.switch = switch
robot.switch.sout.recompute(0)

plug(robot.switch.sout, featureRH_adm.faNufafbDes)


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

# --- Sot controllers ----------------------------------------------------------
robot.sot_pos = SOT('sot_pos')
robot.sot_pos.setSize(robot.dynamic.getDimension())

robot.sot_adm = SOT('sot_adm')
robot.sot_adm.setSize(robot.dynamic.getDimension())

# Plug SOT control to device through control manager
plug(robot.sot_pos.control, robot.controlManager.ctrl_sot_input)
plug(robot.controlManager.u_safe, robot.device.control)
# plug(robot.sot.control, robot.device.control)

# --- PUSH THE TASKS -----------------------------------------------------------

robot.sot_pos.push(robot.taskCom.task.name)
robot.sot_pos.push(robot.contactRF.task.name)
robot.sot_pos.push(robot.contactLF.task.name)
robot.sot_pos.push(robot.taskWaist.task.name)
robot.sot_pos.push(robot.taskPosture.name)

robot.sot_adm.push(robot.taskCom.task.name)
robot.sot_adm.push(robot.contactRF.task.name)
robot.sot_adm.push(robot.contactLF.task.name)
robot.sot_adm.push(robot.taskWaist.task.name)
robot.sot_adm.push(robot.taskRightHandAdm.name)
robot.sot_adm.push(robot.taskRightHandPos.name)
robot.sot_adm.push(robot.taskPosture.name)

# # --- ROS PUBLISHER ----------------------------------------------------------

robot.publisher = create_rospublish(robot, 'robot_publisher')
create_topic(robot.publisher, robot.taskRightHandAdm, 'error', robot=robot, data_type='vector')
create_topic(robot.publisher, robot.controller, 'w_force', robot=robot, data_type='vector')
create_topic(robot.publisher, robot.controller, 'force', robot=robot, data_type='vector')
create_topic(robot.publisher, robot.controller, 'dq', robot=robot, data_type='vector')
create_topic(robot.publisher, robot.controller, 'w_dq', robot=robot, data_type='vector')
create_topic(robot.publisher, robot.controller, 'w_forceDes', robot=robot, data_type='vector')
create_topic(robot.publisher, robot.forceCalibrator, 'leftWristForceOut', robot=robot, data_type='vector')  # calibrated left wrench
create_topic(robot.publisher, robot.forceCalibrator, 'rightWristForceOut', robot=robot, data_type='vector')  # calibrated right wrench

# --- TRACER  ------------------------------------------------------------------

robot.tracer = TracerRealTime("force_tracer")
robot.tracer.setBufferSize(80 * (2**20))
robot.tracer.open('/tmp', 'dg_', '.dat')
robot.device.after.addSignal('{0}.triger'.format(robot.tracer.name))

addTrace(robot.tracer, robot.controller, 'force')
addTrace(robot.tracer, robot.controller, 'w_force')
addTrace(robot.tracer, robot.controller, 'w_dq')
addTrace(robot.tracer, robot.controller, 'dq')
addTrace(robot.tracer, robot.taskRightHandAdm, 'error')

robot.tracer.start()
