import sot_talos_balance.talos.parameter_server_conf as paramServerConfig
import sot_talos_balance.talos.base_estimator_conf as baseEstimatorConf
from sot_talos_balance.create_entities_utils import *
from dynamic_graph.sot.core.meta_tasks_kine import MetaTaskKine6d
from dynamic_graph.sot.core import SOT, Task, GainAdaptive, FeaturePosture
from dynamic_graph import plug
from dynamic_graph.sot.core.matrix_util import matrixToTuple
from dynamic_graph.ros import RosSubscribe, RosPublish
import numpy as np

robot.timeStep = robot.device.getTimeStep()

# --- PARAM_SERVER ---
robot.param_server = create_parameter_server(paramServerConfig, robot.timeStep)
robot.device_filters = create_device_filters(robot, robot.timeStep)
robot.imu_filters = create_imu_filters(robot, robot.timeStep)
robot.baseEstimator = create_base_estimator(robot, robot.timeStep, baseEstimatorConf)

# --- ADMITTANCE CONTROLLER ---
robot.controller = create_end_effector_admittance_controller(robot,'rightWrist')

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
plug(robot.controller.dq, taskRightHand.featureDes.velocity)

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

robot.taskPosture.add(robot.taskPosture.feature.name)
plug(robot.dynamic.position, robot.taskPosture.feature.state)

robot.sot = SOT('sot')
robot.sot.setSize(robot.dynamic.getDimension())
plug(robot.sot.control, robot.device.control)

robot.sot.push(robot.taskPosture.name)
robot.sot.push(taskRightHand.task.name)

# # --- ROS PUBLISHER
robot.publisher = create_rospublish(robot, 'robot_publisher')
create_topic(robot.publisher, robot.controller, 'w_force', robot=robot, data_type='vector')
create_topic(robot.publisher, robot.controller, 'force', robot=robot, data_type='vector')
create_topic(robot.publisher, robot.controller, 'dq', robot=robot, data_type='vector')
create_topic(robot.publisher, robot.controller, 'w_dq', robot=robot, data_type='vector')
create_topic(robot.publisher, robot.controller, 'w_forceDes', robot=robot, data_type='vector')

# # --- ROS SUBSCRIBER
robot.subscriber = RosSubscribe("end_effector_subscriber")
robot.subscriber.add("vector", "w_force", "/sot/controller/w_force")
robot.subscriber.add("vector", "force", "/sot/controller/force")
robot.subscriber.add("vector", "dq", "/sot/controller/dq")
robot.subscriber.add("vector", "w_dq", "/sot/controller/w_dq")
robot.subscriber.add("vector", "w_forceDes", "/sot/controller/w_forceDes")
