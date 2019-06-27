#  From Sot Talos Balance
import sot_talos_balance.talos.parameter_server_conf   as paramServerConfig
import sot_talos_balance.talos.control_manager_conf    as controlManagerConf
import sot_talos_balance.talos.base_estimator_conf     as baseEstimatorConf
import sot_talos_balance.talos.ft_calibration_conf     as forceConf
from sot_talos_balance.simple_admittance_controller import SimpleAdmittanceController
from sot_talos_balance.saturation import Saturation
from sot_talos_balance.meta_task_joint import MetaTaskKineJoint
from sot_talos_balance.create_entities_utils import *

# From Dynamic Graph
from dynamic_graph.sot.core.meta_tasks_kine import MetaTaskKine6d, MetaTaskKineCom
from dynamic_graph.sot.core import Task, FeaturePosture
from dynamic_graph.sot.core.operator import Selec_of_vector, Multiply_double_vector
from dynamic_graph import plug
from dynamic_graph.sot.core import SOT
from dynamic_graph.tracer_real_time import TracerRealTime
from dynamic_graph.ros import RosSubscribe, RosPublish
from dynamic_graph.sot.dynamics_pinocchio import DynamicPinocchio

# Other
from math import sqrt, pi
import numpy as np

robot.timeStep = robot.device.getTimeStep()
timeStep = robot.timeStep

# -------------------------- PARAMETERS ----------------------------------------
# --- Parameters
robotName='robot'
robot.dynamic.com.recompute(0)
dimension = robot.dynamic.getDimension()
mass = robot.dynamic.data.mass[0]
h = robot.dynamic.com.value[2]
g = 9.81
omega = sqrt(g/h)

kSat = 10.0
qLimSat = [pi]
dqLimSat = [10.]

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

KpRoll = [0.0001]
KpPitch = [0.0001]

# --- RIGHT ANKLE PITCH
controller = SimpleAdmittanceController("rightPitchAnkleController")
controller.Kp.value = KpPitch

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

robot.rightPitchSelec = Selec_of_vector("rightPitchSelec")
robot.rightPitchSelec.selec(4, 5)
plug(robot.wrenchDistributor.ankleWrenchRight, robot.rightPitchSelec.sin)

# robot.multRP = Multiply_double_vector("multRP")
# robot.multRP.sin1.value = 1.0;
# plug(robot.rightPitchSelec.sout, robot.multRP.sin2)
# plug(robot.multRP.sout, robot.rightPitchAnkleController.tauDes)
plug(robot.rightPitchSelec.sout, robot.rightPitchAnkleController.tauDes)

robot.rightPitchSaturation = Saturation("rightPitchSaturation")
plug(robot.stateselecRP.sout, robot.rightPitchSaturation.x)
plug(robot.rightPitchAnkleController.dqRef, robot.rightPitchSaturation.y)
robot.rightPitchSaturation.k.value = kSat
robot.rightPitchSaturation.xLim.value = qLimSat
robot.rightPitchSaturation.yLim.value = dqLimSat

robot.rightAnklePitchTask = MetaTaskKineJoint(robot.dynamic, RightPitchJoint+6)
robot.rightAnklePitchTask.task.controlGain.value = 0
robot.rightAnklePitchTask.task.setWithDerivative(True)
plug(robot.rightPitchAnkleController.qRef, robot.rightAnklePitchTask.featureDes.errorIN)
plug(robot.rightPitchSaturation.yOut, robot.rightAnklePitchTask.featureDes.errordotIN)

# --- RIGHT ANKLE ROLL
controller = SimpleAdmittanceController("rightRollAnkleController")
controller.Kp.value = KpRoll

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

robot.rightRollSelec = Selec_of_vector("rightRollSelec")
robot.rightRollSelec.selec(3, 4)
plug(robot.wrenchDistributor.ankleWrenchRight, robot.rightRollSelec.sin)

# robot.multRR = Multiply_double_vector("multRR")
# robot.multRR.sin1.value = 1.0;
# plug(robot.rightRollSelec.sout, robot.multRR.sin2)
# plug(robot.multRR.sout, robot.rightRollAnkleController.tauDes)
plug(robot.rightRollSelec.sout, robot.rightRollAnkleController.tauDes)

robot.rightRollSaturation = Saturation("rightRollSaturation")
plug(robot.stateselecRR.sout, robot.rightRollSaturation.x)
plug(robot.rightRollAnkleController.dqRef, robot.rightRollSaturation.y)
robot.rightRollSaturation.k.value = kSat
robot.rightRollSaturation.xLim.value = qLimSat
robot.rightRollSaturation.yLim.value = dqLimSat

robot.rightAnkleRollTask = MetaTaskKineJoint(robot.dynamic, RightRollJoint+6)
robot.rightAnkleRollTask.task.controlGain.value = 0
robot.rightAnkleRollTask.task.setWithDerivative(True)
plug(robot.rightRollAnkleController.qRef, robot.rightAnkleRollTask.featureDes.errorIN)
plug(robot.rightRollSaturation.yOut, robot.rightAnkleRollTask.featureDes.errordotIN)

# --- LEFT ANKLE PITCH
controller = SimpleAdmittanceController("leftPitchAnkleController")
controller.Kp.value = KpPitch

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

robot.leftPitchSelec = Selec_of_vector("leftPitchSelec")
robot.leftPitchSelec.selec(4, 5)
plug(robot.wrenchDistributor.ankleWrenchLeft, robot.leftPitchSelec.sin)

# robot.multLP = Multiply_double_vector("multLP")
# robot.multLP.sin1.value = 1.0;
# plug(robot.leftPitchSelec.sout, robot.multLP.sin2)
# plug(robot.multLP.sout, robot.leftPitchAnkleController.tauDes)
plug(robot.leftPitchSelec.sout, robot.leftPitchAnkleController.tauDes)


robot.leftPitchSaturation = Saturation("leftPitchSaturation")
plug(robot.stateselecLP.sout, robot.leftPitchSaturation.x)
plug(robot.leftPitchAnkleController.dqRef, robot.leftPitchSaturation.y)
robot.leftPitchSaturation.k.value = kSat
robot.leftPitchSaturation.xLim.value = qLimSat
robot.leftPitchSaturation.yLim.value = dqLimSat

robot.leftAnklePitchTask = MetaTaskKineJoint(robot.dynamic, LeftPitchJoint+6)
robot.leftAnklePitchTask.task.controlGain.value = 0
robot.leftAnklePitchTask.task.setWithDerivative(True)
plug(robot.leftPitchAnkleController.qRef, robot.leftAnklePitchTask.featureDes.errorIN)
plug(robot.leftPitchSaturation.yOut, robot.leftAnklePitchTask.featureDes.errordotIN)

# --- LEFT ANKLE ROLL
controller = SimpleAdmittanceController("leftRollAnkleController")
controller.Kp.value = KpRoll

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

robot.leftRollSelec = Selec_of_vector("leftRollSelec")
robot.leftRollSelec.selec(3, 4)
plug(robot.wrenchDistributor.ankleWrenchLeft, robot.leftRollSelec.sin)
plug(robot.leftRollSelec.sout, robot.leftRollAnkleController.tauDes)

# robot.multLR = Multiply_double_vector("multLR")
# robot.multLR.sin1.value = 1.0;
# plug(robot.leftRollSelec.sout, robot.multLR.sin2)
# plug(robot.multLR.sout, robot.leftRollAnkleController.tauDes)
plug(robot.leftRollSelec.sout, robot.leftRollAnkleController.tauDes)

robot.leftRollSaturation = Saturation("leftRollSaturation")
plug(robot.stateselecLR.sout, robot.leftRollSaturation.x)
plug(robot.leftRollAnkleController.dqRef, robot.leftRollSaturation.y)
robot.leftRollSaturation.k.value = kSat
robot.leftRollSaturation.xLim.value = qLimSat
robot.leftRollSaturation.yLim.value = dqLimSat

robot.leftAnkleRollTask = MetaTaskKineJoint(robot.dynamic, LeftRollJoint+6)
robot.leftAnkleRollTask.task.controlGain.value = 0
robot.leftAnkleRollTask.task.setWithDerivative(True)
plug(robot.leftRollAnkleController.qRef, robot.leftAnkleRollTask.featureDes.errorIN)
plug(robot.leftRollSaturation.yOut, robot.leftAnkleRollTask.featureDes.errordotIN)

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
plug(robot.sot.control, robot.controlManager.ctrl_sot_input)
plug(robot.controlManager.u_safe, robot.device.control)

robot.device.control.recompute(0)

# -------------------------- PLOTS --------------------------
# --- ROS PUBLISHER
robot.publisher = create_rospublish(robot, 'robot_publisher')        

create_topic(robot.publisher, robot.rightPitchSaturation, 'yOut', robot = robot, data_type='vector')
create_topic(robot.publisher, robot.rightRollSaturation, 'yOut', robot = robot, data_type='vector')
create_topic(robot.publisher, robot.leftPitchSaturation, 'yOut', robot = robot, data_type='vector')
create_topic(robot.publisher, robot.leftRollSaturation, 'yOut', robot = robot, data_type='vector')

create_topic(robot.publisher, robot.rightPitchAnkleController, 'tau', robot = robot, data_type='vector')
create_topic(robot.publisher, robot.rightRollAnkleController, 'tau', robot = robot, data_type='vector')
create_topic(robot.publisher, robot.leftPitchAnkleController, 'tau', robot = robot, data_type='vector')
create_topic(robot.publisher, robot.leftRollAnkleController, 'tau', robot = robot, data_type='vector')

create_topic(robot.publisher, robot.rightPitchAnkleController, 'tauDes', robot = robot, data_type='vector')
create_topic(robot.publisher, robot.rightRollAnkleController, 'tauDes', robot = robot, data_type='vector')
create_topic(robot.publisher, robot.leftPitchAnkleController, 'tauDes', robot = robot, data_type='vector')
create_topic(robot.publisher, robot.leftRollAnkleController, 'tauDes', robot = robot, data_type='vector')


# # --- ROS SUBSCRIBER
robot.subscriber = RosSubscribe("ankle_joint_subscriber")

robot.subscriber.add("vector", "dqRefRP", "/sot/rightPitchSaturation/yOut")
robot.subscriber.add("vector", "dqRefRR", "/sot/rightRollSaturation/yOut")
robot.subscriber.add("vector", "dqRefLP", "/sot/leftPitchSaturation/yOut")
robot.subscriber.add("vector", "dqRefLR", "/sot/leftRollSaturation/yOut")

robot.subscriber.add("vector", "tauRP", "/sot/rightPitchAnkleController/tau")
robot.subscriber.add("vector", "tauRR", "/sot/rightRollAnkleController/tau")
robot.subscriber.add("vector", "tauLP", "/sot/leftPitchAnkleController/tau")
robot.subscriber.add("vector", "tauLR", "/sot/leftRollAnkleController/tau")

robot.subscriber.add("vector", "tauDesRP", "/sot/rightPitchAnkleController/tauDes")
robot.subscriber.add("vector", "tauDesRR", "/sot/rightRollAnkleController/tauDes")
robot.subscriber.add("vector", "tauDesLP", "/sot/leftPitchAnkleController/tauDes")
robot.subscriber.add("vector", "tauDesLR", "/sot/leftRollAnkleController/tauDes")
