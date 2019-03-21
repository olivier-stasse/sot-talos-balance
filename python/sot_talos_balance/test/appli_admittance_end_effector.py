import sot_talos_balance.talos.parameter_server_conf as paramServerConfig
from sot_talos_balance.create_entities_utils import *
from dynamic_graph.sot.core.meta_tasks_kine import MetaTaskKine6d, MetaTaskKineCom
from dynamic_graph import plug
from dynamic_graph.sot.core import SOT
from dynamic_graph.sot.core.matrix_util import matrixToTuple
from numpy import eye
import numpy as np

robot.timeStep = robot.device.getTimeStep()
timeStep = robot.timeStep

# --- CREATE PARAM_SERVER
robot.param_server = create_parameter_server(paramServerConfig, timeStep)

# --- ADMITTANCE CONTROLLER ---
Kp = np.array([0.0, 0., 0., 0., 0., 0.])
robot.admittanceController = create_end_effector_admittance_controller(Kp, timeStep, robot)
robot.admittanceController.forceDes.value = [0., 0., 0., 0., 0., 0.]

# --- HAND TASK ---
taskRightHand = MetaTaskKine6d('rh', robot.dynamic, 'rh', robot.OperationalPointsMap['right-wrist'])
handMgrip = np.eye(4)
handMgrip[0:3, 3] = (0.1, 0, 0)
taskRightHand.opmodif = matrixToTuple(handMgrip)
taskRightHand.feature.frame('current')
taskRightHand.feature.selec.value = '000111'
taskRightHand.task.setWithDerivative(True)
taskRightHand.task.controlGain.value = 0
taskRightHand.feature.position.value = np.eye(4)
taskRightHand.feature.velocity.value = [0., 0., 0., 0., 0., 0.]
taskRightHand.featureDes.position.value = np.eye(4)
plug(robot.admittanceController.dqRef, taskRightHand.featureDes.velocity)

# --- POSTURE TASK ---
robot.taskPosture = Task('task_posture')
robot.taskPosture.controlGain.value = 100.0
robot.taskPosture.feature = FeaturePosture('feature_posture')

q = list(robot.dynamic.position.value)
robot.taskPosture.feature.state.value = q
robot.taskPosture.feature.posture.value = q

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

robot.taskPosture.feature.selectDof(36, True)
robot.taskPosture.feature.selectDof(37, True)

robot.taskPosture.add(robot.taskPosture.feature.name)
plug(robot.dynamic.position, robot.taskPosture.feature.state)

robot.sot = SOT('sot')
robot.sot.setSize(robot.dynamic.getDimension())
plug(robot.sot.control, robot.device.control)

robot.sot.push(robot.taskPosture.name)
robot.sot.push(taskRightHand.task.name)

# # --- ROS PUBLISHER
robot.publisher = create_rospublish(robot, 'robot_publisher')
create_topic(robot.publisher, robot.admittanceController, 'forceWorldFrame', robot=robot, data_type='vector')
create_topic(robot.publisher, robot.admittanceController, 'force', robot=robot, data_type='vector')
create_topic(robot.publisher, robot.admittanceController, 'dqRef', robot=robot, data_type='vector')
create_topic(robot.publisher, robot.admittanceController, 'forceDes', robot=robot, data_type='vector')

# # --- ROS SUBSCRIBER
robot.subscriber = RosSubscribe("end_effector_subscriber")
robot.subscriber.add("vector", "forceWorldFrame", "/sot/admittanceController/forceWorldFrame")
robot.subscriber.add("vector", "force", "/sot/admittanceController/force")
robot.subscriber.add("vector", "dqRef", "/sot/admittanceController/dqRef")
robot.subscriber.add("vector", "forceDes", "/sot/admittanceController/forceDes")
