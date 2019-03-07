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
# Create task
taskRightHand = MetaTaskKine6d('rh', robot.dynamic, 'rh', robot.OperationalPointsMap['right-wrist'])
handMgrip = eye(4)
handMgrip[0:3, 3] = (0.1, 0, 0)
taskRightHand.opmodif = matrixToTuple(handMgrip)
taskRightHand.feature.frame('current')
taskRightHand.feature.selec.value = '000111'
taskRightHand.task.setWithDerivative(True)
taskRightHand.task.controlGain.value = 0
taskRightHand.feature.position.value =  eye(4)
taskRightHand.feature.velocity.value =  [0., 0., 0., 0., 0., 0.]
taskRightHand.featureDes.position.value =  eye(4)
plug(robot.admittanceController.dqRef, taskRightHand.featureDes.velocity)

# --- CONTACTS
# Define contactLF and contactRF
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

robot.sot = SOT('sot')
robot.sot.setSize(robot.dynamic.getDimension())
plug(robot.sot.control, robot.device.control)

robot.sot.push(robot.contactRF.task.name)
robot.sot.push(robot.contactLF.task.name)
robot.device.control.recompute(0)

