#  From Sot Talos Balance
import sot_talos_balance.talos.parameter_server_conf   as paramServerConfig
import sot_talos_balance.talos.control_manager_conf    as controlManagerConf
import sot_talos_balance.talos.base_estimator_conf     as baseEstimatorConf
import sot_talos_balance.talos.ft_calibration_conf     as forceConf
from sot_talos_balance.simple_admittance_controller import SimpleAdmittanceController
from sot_talos_balance.meta_task_joint import MetaTaskKineJoint
from sot_talos_balance.create_entities_utils import *

# From Dynamic Graph
from dynamic_graph.sot.core.meta_tasks_kine import MetaTaskKine6d, MetaTaskKineCom
from dynamic_graph.sot.core import Task, FeaturePosture
from dynamic_graph.sot.core.operator import Selec_of_vector
from dynamic_graph import plug
from dynamic_graph.sot.core import SOT
from dynamic_graph.tracer_real_time import TracerRealTime
from dynamic_graph.sot.dynamics_pinocchio import DynamicPinocchio

# Other
from math import sqrt
import numpy as np

robot.timeStep = robot.device.getTimeStep()
timeStep = robot.timeStep

# -------------------------- PARAMETERS ----------------------------------------
# --- Pendulum parameters
robotName='robot'
robot.dynamic.com.recompute(0)
dimension = robot.dynamic.getDimension()
mass = robot.dynamic.data.mass[0]
h = robot.dynamic.com.value[2]
g = 9.81
omega = sqrt(g/h)

# --- Parameter server
robot.paramServer = create_parameter_server(paramServerConfig, timeStep)

# --- Initial feet and waist
robot.dynamic.createOpPoint('LF',robot.OperationalPointsMap['left-ankle'])
robot.dynamic.createOpPoint('RF',robot.OperationalPointsMap['right-ankle'])
robot.dynamic.createOpPoint('WT',robot.OperationalPointsMap['waist'])
robot.dynamic.LF.recompute(0)
robot.dynamic.RF.recompute(0)
robot.dynamic.WT.recompute(0)

# -------------------------- DESIRED TRAJECTORY --------------------------------
# --- Trajectory generators
robot.COMTrajectoryGenerator = create_com_trajectory_generator(timeStep, robot)

robot.leftFootTrajectoryGenerator  = create_pose_rpy_trajectory_generator(timeStep, robot, 'LF')
robot.leftFootToMatrix = PoseRollPitchYawToMatrixHomo('leftFoot2m')
plug(robot.leftFootTrajectoryGenerator.x, robot.leftFootToMatrix.sin)

robot.rightFootTrajectoryGenerator  = create_pose_rpy_trajectory_generator(timeStep, robot, 'RF')
robot.rightFootToMatrix = PoseRollPitchYawToMatrixHomo('rightFoot2m')
plug(robot.rightFootTrajectoryGenerator.x, robot.rightFootToMatrix.sin)

robot.waistTrajectoryGenerator = create_orientation_rpy_trajectory_generator(timeStep, robot, 'WT')
robot.waistMix = Mix_of_vector("waistMix")
robot.waistMix.setSignalNumber(3)
robot.waistMix.addSelec(1, 0, 3)
robot.waistMix.addSelec(2, 3, 3)
robot.waistMix.default.value = [0.0]*6
robot.waistMix.signal("sin1").value = [0.0]*3
plug(robot.waistTrajectoryGenerator.x, robot.waistMix.signal("sin2"))
robot.waistToMatrix = PoseRollPitchYawToMatrixHomo('w2m')
plug(robot.waistMix.sout, robot.waistToMatrix.sin)

# --- Interface with controller entities
PG = DummyWalkingPatternGenerator('dummy_PG')
PG.init()
PG.omega.value = omega
plug(robot.waistToMatrix.sout, PG.waist)
plug(robot.leftFootToMatrix.sout, PG.footLeft)
plug(robot.rightFootToMatrix.sout, PG.footRight)
plug(robot.COMTrajectoryGenerator.x, PG.com)
plug(robot.COMTrajectoryGenerator.dx, PG.vcom)
plug(robot.COMTrajectoryGenerator.ddx, PG.acom)
robot.PG = PG

# --- Compute the values to use them in initialization
robot.PG.comDes.recompute(0)
robot.PG.dcmDes.recompute(0)
robot.PG.zmpDes.recompute(0)

# -------------------------- ESTIMATION ----------------------------------------
# --- Base Estimation
robot.device_filters = create_device_filters(robot, timeStep)
robot.imu_filters    = create_imu_filters(robot, timeStep)
robot.baseEstimator  = create_base_estimator(robot, timeStep, baseEstimatorConf)

# --- Reference frame
referenceFrame = SimpleReferenceFrame('referenceFrame')
referenceFrame.init(robotName)
plug(robot.dynamic.LF, referenceFrame.footLeft)
plug(robot.dynamic.RF, referenceFrame.footRight)
robot.referenceFrame = referenceFrame

# --- State transformation
stateTransform = StateTransformation("stateTransform")
stateTransform.init()
plug(robot.referenceFrame.referenceFrame, stateTransform.referenceFrame)
plug(robot.baseEstimator.q, stateTransform.q_in)
plug(robot.baseEstimator.v, stateTransform.v_in)
robot.stateTransform = stateTransform

# --- Conversion
e2q = EulerToQuat('e2q')
plug(robot.stateTransform.q, e2q.euler)
robot.e2q = e2q

# --- Kinematic computations
robot.rdynamic = DynamicPinocchio("real_dynamics")
robot.rdynamic.setModel(robot.dynamic.model)
robot.rdynamic.setData(robot.rdynamic.model.createData())
plug(robot.stateTransform.q,robot.rdynamic.position)
robot.rdynamic.velocity.value = [0.0]*dimension
robot.rdynamic.acceleration.value = [0.0]*dimension

# --- CoM Estimation
dcmEstimator = DcmEstimator('dcmEstimator')
dcmEstimator.init(timeStep, robotName)
plug(robot.e2q.quaternion, dcmEstimator.q)
plug(robot.stateTransform.v, dcmEstimator.v)
robot.dcmEstimator = dcmEstimator

# --- DCM Estimation
estimator = DummyDcmEstimator("dummy")
estimator.omega.value = omega
estimator.mass.value = 1.0
plug(robot.dcmEstimator.c, estimator.com)
plug(robot.dcmEstimator.dc,estimator.momenta)
estimator.init()
robot.estimator = estimator

# --- Force calibration
robot.ftc = create_ft_calibrator(robot, forceConf)

# --- ZMP estimation
zmpEstimator = SimpleZmpEstimator("zmpEst")
robot.rdynamic.createOpPoint('sole_LF','left_sole_link')
robot.rdynamic.createOpPoint('sole_RF','right_sole_link')
plug(robot.rdynamic.sole_LF,zmpEstimator.poseLeft)
plug(robot.rdynamic.sole_RF,zmpEstimator.poseRight)
plug(robot.ftc.left_foot_force_out,zmpEstimator.wrenchLeft)
plug(robot.ftc.right_foot_force_out,zmpEstimator.wrenchRight)
zmpEstimator.init()
robot.zmpEstimator = zmpEstimator

# -------------------------- DCM CONTROLLER ------------------------------------
Kp_dcm = [5.0,5.0,5.0]
Ki_dcm = [0.0,0.0,0.0]
gamma_dcm = 0.2

dcmController = DcmController("dcmCtrl")

dcmController.Kp.value = Kp_dcm
dcmController.Ki.value = Ki_dcm
dcmController.decayFactor.value = gamma_dcm
dcmController.mass.value = mass
dcmController.omega.value = omega

plug(robot.dcmEstimator.c, dcmController.com)
plug(robot.estimator.dcm, dcmController.dcm)

plug(robot.PG.zmpDes, dcmController.zmpDes)
plug(robot.PG.dcmDes, dcmController.dcmDes)

dcmController.init(timeStep)

robot.dcmControl = dcmController

# -------------------------- WRENCH DISTRIBUTOR --------------------------------
distribute = create_distribute_wrench(baseEstimatorConf)
plug(robot.e2q.quaternion, distribute.q)
plug(robot.dcmControl.wrenchRef, distribute.wrenchDes)
distribute.init(robotName)
robot.wrenchDistributor = distribute

# -------------------------- ANKLE ADMITTANCE ----------------------------------
RightPitchJoint = 10
RightRollJoint = 11
LeftPitchJoint = 4
LeftRollJoint = 5

Kp = [0.0005]

# --- RIGHT ANKLE PITCH
controller = SimpleAdmittanceController("rightPitchAnkleController")
controller.Kp.value = Kp

robot.stateselecRP = Selec_of_vector("stateselecRP")
robot.stateselecRP.selec(RightPitchJoint+6, RightPitchJoint+7)
plug(robot.device.state, robot.stateselecRP.sin)
plug(robot.stateselecRP.sout, controller.state)

robot.tauselecRP = Selec_of_vector("tauselecRP")
robot.tauselecRP.selec(RightPitchJoint, RightPitchJoint+1)
plug(robot.device.ptorque, robot.tauselecRP.sin)
plug(robot.tauselecRP.sout, controller.tau)

controller.tauDes.value = [0.0]
controller.init(timeStep, 1)
controller.setPosition([robot.device.state.value[RightPitchJoint+6]])
robot.rightPitchAnkleController = controller

rightPitchSelec = Selec_of_vector("rightPitchSelec")
rightPitchSelec.selec(4, 5)
plug(robot.wrenchDistributor.ankleWrenchRight, rightPitchSelec.sin)
plug(rightPitchSelec.sout, robot.rightPitchAnkleController.tauDes)

robot.rightAnklePitchTask = MetaTaskKineJoint(robot.dynamic, RightPitchJoint+6)
robot.rightAnklePitchTask.task.controlGain.value = 0
robot.rightAnklePitchTask.task.setWithDerivative(True)
plug(robot.rightPitchAnkleController.qRef, robot.rightAnklePitchTask.featureDes.errorIN)
plug(robot.rightPitchAnkleController.dqRef, robot.rightAnklePitchTask.featureDes.errordotIN)

# --- RIGHT ANKLE ROLL
controller = SimpleAdmittanceController("rightRollAnkleController")
controller.Kp.value = Kp

robot.stateselecRR = Selec_of_vector("stateselecRR")
robot.stateselecRR.selec(RightRollJoint+6, RightRollJoint+7)
plug(robot.device.state, robot.stateselecRR.sin)
plug(robot.stateselecRR.sout, controller.state)

robot.tauselecRR = Selec_of_vector("tauselecRR")
robot.tauselecRR.selec(RightRollJoint, RightRollJoint+1)
plug(robot.device.ptorque, robot.tauselecRR.sin)
plug(robot.tauselecRR.sout, controller.tau)

controller.tauDes.value = [0.0]
controller.init(timeStep, 1)
controller.setPosition([robot.device.state.value[RightRollJoint+6]])
robot.rightRollAnkleController = controller

rightRollSelec = Selec_of_vector("rightRollSelec")
rightRollSelec.selec(3, 4)
plug(robot.wrenchDistributor.ankleWrenchRight, rightRollSelec.sin)
plug(rightRollSelec.sout, robot.rightRollAnkleController.tauDes)

robot.rightAnkleRollTask = MetaTaskKineJoint(robot.dynamic, RightRollJoint+6)
robot.rightAnkleRollTask.task.controlGain.value = 0
robot.rightAnkleRollTask.task.setWithDerivative(True)
plug(robot.rightRollAnkleController.qRef, robot.rightAnkleRollTask.featureDes.errorIN)
plug(robot.rightRollAnkleController.dqRef, robot.rightAnkleRollTask.featureDes.errordotIN)

# --- LEFT ANKLE PITCH
controller = SimpleAdmittanceController("leftPitchAnkleController")
controller.Kp.value = Kp

robot.stateselecLP = Selec_of_vector("stateselecLP")
robot.stateselecLP.selec(LeftPitchJoint+6, LeftPitchJoint+7)
plug(robot.device.state, robot.stateselecLP.sin)
plug(robot.stateselecLP.sout, controller.state)

robot.tauselecLP = Selec_of_vector("tauselecLP")
robot.tauselecLP.selec(LeftPitchJoint, LeftPitchJoint+1)
plug(robot.device.ptorque, robot.tauselecLP.sin)
plug(robot.tauselecLP.sout, controller.tau)

controller.tauDes.value = [0.0]
controller.init(timeStep, 1)
controller.setPosition([robot.device.state.value[LeftPitchJoint+6]])
robot.leftPitchAnkleController = controller

leftPitchSelec = Selec_of_vector("leftPitchSelec")
leftPitchSelec.selec(4, 5)
plug(robot.wrenchDistributor.ankleWrenchLeft, leftPitchSelec.sin)
plug(leftPitchSelec.sout, robot.leftPitchAnkleController.tauDes)

robot.leftAnklePitchTask = MetaTaskKineJoint(robot.dynamic, LeftPitchJoint+6)
robot.leftAnklePitchTask.task.controlGain.value = 0
robot.leftAnklePitchTask.task.setWithDerivative(True)
plug(robot.leftPitchAnkleController.qRef,robot.leftAnklePitchTask.featureDes.errorIN)
plug(robot.leftPitchAnkleController.dqRef,robot.leftAnklePitchTask.featureDes.errordotIN)

# --- LEFT ANKLE ROLL
controller = SimpleAdmittanceController("leftRollAnkleController")
controller.Kp.value = Kp

robot.stateselecLR = Selec_of_vector("stateselecLP")
robot.stateselecLR.selec(LeftRollJoint+6, LeftRollJoint+7)
plug(robot.device.state, robot.stateselecLR.sin)
plug(robot.stateselecLR.sout, controller.state)

robot.tauselecLR = Selec_of_vector("tauselecLP")
robot.tauselecLR.selec(LeftRollJoint, LeftRollJoint+1)
plug(robot.device.ptorque, robot.tauselecLR.sin)
plug(robot.tauselecLR.sout, controller.tau)

controller.tauDes.value = [0.0]
controller.init(timeStep, 1)
controller.setPosition([robot.device.state.value[LeftRollJoint+6]])
robot.leftRollAnkleController = controller

leftRollSelec = Selec_of_vector("leftRollSelec")
leftRollSelec.selec(3, 4)
plug(robot.wrenchDistributor.ankleWrenchLeft, leftRollSelec.sin)
plug(leftRollSelec.sout, robot.leftRollAnkleController.tauDes)

robot.leftAnkleRollTask = MetaTaskKineJoint(robot.dynamic, LeftRollJoint+6)
robot.leftAnkleRollTask.task.controlGain.value = 0
robot.leftAnkleRollTask.task.setWithDerivative(True)
plug(robot.leftRollAnkleController.qRef, robot.leftAnkleRollTask.featureDes.errorIN)
plug(robot.leftRollAnkleController.dqRef, robot.leftAnkleRollTask.featureDes.errordotIN)

# -------------------------- CONTROL MANAGER -----------------------------------

robot.controlManager = create_ctrl_manager(controlManagerConf, timeStep, robot_name='robot')
robot.controlManager.addCtrlMode('sot_input')
robot.controlManager.setCtrlMode('all','sot_input')
robot.controlManager.addEmergencyStopSIN('zmp')

# -------------------------- SOT CONTROL ---------------------------------------
# --- Posture
robot.taskPosture = Task ('taskPosture')
robot.taskPosture.feature = FeaturePosture('featurePosture')

q = list(robot.dynamic.position.value)
robot.taskPosture.feature.state.value = q
robot.taskPosture.feature.posture.value = q

for i in range(6, 38):
  robot.taskPosture.feature.selectDof(i, True)

robot.taskPosture.controlGain.value = 100.0
robot.taskPosture.add(robot.taskPosture.feature.name)
plug(robot.dynamic.position, robot.taskPosture.feature.state)

# --- COM Height
robot.taskCom = MetaTaskKineCom(robot.dynamic)
plug(robot.PG.comDes, robot.taskCom.featureDes.errorIN)
robot.taskCom.task.controlGain.value = 10
robot.taskCom.feature.selec.value = '100'

# --- SOT
robot.sot = SOT('sot')
robot.sot.setSize(robot.dynamic.getDimension())
# plug(robot.sot.control, robot.controlManager.ctrl_sot_input)
# plug(robot.controlManager.u_safe, robot.device.control)
plug(robot.sot.control,robot.device.control)

# robot.sot.push(robot.rightAnklePitchTask.task.name)
# robot.sot.push(robot.rightAnkleRollTask.task.name)
# robot.sot.push(robot.leftAnklePitchTask.task.name)
# robot.sot.push(robot.leftAnkleRollTask.task.name)
# robot.sot.push(robot.taskCom.task.name)
# robot.sot.push(robot.taskPosture.name)

robot.device.control.recompute(0)