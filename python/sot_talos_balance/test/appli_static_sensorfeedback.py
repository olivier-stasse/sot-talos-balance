from sot_talos_balance.create_entities_utils import *
from sot_talos_balance.meta_task_config import MetaTaskKineConfig
import sot_talos_balance.talos.parameter_server_conf as param_server_conf
from sot_talos_balance.talos import base_estimator_conf
from dynamic_graph.sot.core.meta_tasks_kine import MetaTaskKine6d, MetaTaskKineCom, gotoNd
from dynamic_graph import plug
from dynamic_graph.sot.core import SOT
from dynamic_graph.sot.core.operator import Selec_of_vector

from dynamic_graph.ros import RosSubscribe
from dynamic_graph.sot.dynamics_pinocchio import DynamicPinocchio

robot.timeStep = robot.device.getTimeStep()
dt = robot.timeStep;

# --- Estimators
robot.param_server            = create_parameter_server(param_server_conf,dt)
robot.device_filters          = create_device_filters(robot, dt)
robot.imu_filters             = create_imu_filters(robot, dt)
robot.base_estimator          = create_base_estimator(robot, dt, base_estimator_conf)
robot.be_filters              = create_be_filters(robot, dt)
robot.dcm_estimator           = create_dcm_estimator(robot, dt)

robot.baseselec = Selec_of_vector("base_selec")
robot.baseselec.selec(0,6)
plug(robot.base_estimator.q,robot.baseselec.sin)

# --- Dynamic pinocchio
robot.rdynamic = DynamicPinocchio("real_dynamics")
robot.rdynamic.setModel(robot.dynamic.model)
robot.rdynamic.setData(robot.rdynamic.model.createData())

plug(robot.device.state,robot.rdynamic.position)
plug(robot.baseselec.sout,robot.rdynamic.ffposition)

robot.rdynamic.velocity.value = [0.0]*robot.dynamic.getDimension()

robot.rdynamic.acceleration.value = [0.0]*robot.dynamic.getDimension()

# --- Posture
#robot.taskPos = MetaTaskKineConfig(robot.rdynamic)
#robot.taskPos.featureDes.errorIN.value = robot.dynamic.position.value
#robot.taskPos.task.controlGain.value = 100
#plug(robot.state,robot.taskPos.feature.errorIN)

# --- COM
robot.taskCom = MetaTaskKineCom(robot.rdynamic)
robot.dynamic.com.recompute(0)
robot.taskCom.featureDes.errorIN.value = robot.dynamic.com.value
robot.taskCom.task.controlGain.value = 100
# plug(robot.dcm_estimator.c, robot.taskCom.feature.errorIN)

# --- CONTACTS
#define contactLF and contactRF
robot.contactLF = MetaTaskKine6d('contactLF',robot.rdynamic,'LF',robot.OperationalPointsMap['left-ankle'])
robot.contactLF.feature.frame('desired')
robot.contactLF.gain.setConstant(100)
# robot.contactLF.keep()
locals()['contactLF'] = robot.contactLF

robot.contactRF = MetaTaskKine6d('contactRF',robot.rdynamic,'RF',robot.OperationalPointsMap['right-ankle'])
robot.contactRF.feature.frame('desired')
robot.contactRF.gain.setConstant(100)
# robot.contactRF.keep()
locals()['contactRF'] = robot.contactRF

# --- SOLVER
robot.sot = SOT('sot')
robot.sot.setSize(robot.dynamic.getDimension())
robot.sot.push(robot.contactRF.task.name)
robot.sot.push(robot.contactLF.task.name)
robot.sot.push(robot.taskCom.task.name)
#robot.sot.push(robot.taskPos.task.name)

robot.device.control.value = [0.0]*robot.dynamic.getDimension()

# --- TRACER
robot.tracer = TracerRealTime("com_tracer")
robot.tracer.setBufferSize(80*(2**20))
robot.tracer.open('/tmp','dg_','.dat')
robot.device.after.addSignal('{0}.triger'.format(robot.tracer.name))
robot.device.after.addSignal('{0}.com'.format(robot.rdynamic.name))
robot.device.after.addSignal('{0}.com'.format(robot.dynamic.name))
robot.device.after.addSignal('{0}.errorIN'.format(robot.taskCom.featureDes.name))

addTrace(robot.tracer, robot.taskCom.featureDes, 'errorIN')
addTrace(robot.tracer, robot.dynamic, 'com')
addTrace(robot.tracer, robot.rdynamic, 'com')

robot.tracer.start()
