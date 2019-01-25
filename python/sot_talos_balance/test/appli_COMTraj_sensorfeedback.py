from sot_talos_balance.create_entities_utils import *
# from sot_talos_balance.meta_task_config import MetaTaskKineConfig
import sot_talos_balance.control_manager_conf as param_server_conf
from sot_talos_balance.talos import base_estimator_conf
from dynamic_graph.sot.core.meta_tasks_kine import MetaTaskKine6d, MetaTaskKineCom, gotoNd
from dynamic_graph import plug
from dynamic_graph.sot.core import SOT

from dynamic_graph.sot.dynamics_pinocchio import DynamicPinocchio

dt = robot.timeStep;

# --- COM trajectory generator
robot.comTrajGen = create_com_trajectory_generator(dt,robot);

# --- Estimators
robot.param_server            = create_parameter_server(param_server_conf,dt)
robot.device_filters          = create_device_filters(robot, dt)
robot.imu_filters             = create_imu_filters(robot, dt)
robot.base_estimator          = create_base_estimator(robot, dt, base_estimator_conf)
robot.be_filters              = create_be_filters(robot, dt)
robot.dcm_estimator           = create_dcm_estimator(robot, dt)

# --- Dynamic pinocchio
robot.rdynamic = DynamicPinocchio("real_dynamics")
robot.rdynamic.setModel(robot.dynamic.model)
robot.rdynamic.setData(robot.rdynamic.model.createData())

plug(robot.base_estimator.q,robot.rdynamic.position)

robot.rdynamic.velocity.value = [0.0]*38

robot.rdynamic.acceleration.value = [0.0]*38

# --- Posture
#robot.taskPos = MetaTaskKineConfig(robot.dynamic)
#robot.taskPos.featureDes.errorIN.value = robot.dynamic.position.value
#robot.taskPos.task.controlGain.value = 10
#plug(robot.base_estimator.q,robot.taskPos.feature.errorIN)

# --- COM
robot.taskCom = MetaTaskKineCom(robot.rdynamic)
robot.dynamic.com.recompute(0)
robot.taskCom.featureDes.errorIN.value = robot.dynamic.com.value
robot.taskCom.task.controlGain.value = 10
plug(robot.dcm_estimator.c,robot.taskCom.feature.errorIN)

# --- CONTACTS
#define contactLF and contactRF
robot.contactLF = MetaTaskKine6d('contactLF',robot.dynamic,'LF',robot.OperationalPointsMap['left-ankle'])
robot.contactLF.feature.frame('desired')
robot.contactLF.gain.setConstant(100)
robot.contactLF.keep()
locals()['contactLF'] = robot.contactLF

robot.contactRF = MetaTaskKine6d('contactRF',robot.dynamic,'RF',robot.OperationalPointsMap['right-ankle'])
robot.contactRF.feature.frame('desired')
robot.contactRF.gain.setConstant(100)
robot.contactRF.keep()
locals()['contactRF'] = robot.contactRF

# --- SOLVER
robot.sot = SOT('sot')
robot.sot.setSize(robot.dynamic.getDimension())
plug(robot.sot.control,robot.device.control)
# plug(robot.comTrajGen.x,    robot.taskCom.featureDes.errorIN) # this is done after the SOT has been started

robot.sot.push(robot.contactRF.task.name)
robot.sot.push(robot.contactLF.task.name)
robot.sot.push(robot.taskCom.task.name)
#robot.sot.push(robot.taskPos.task.name)

