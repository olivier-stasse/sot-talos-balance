from sot_talos_balance.create_entities_utils import *
from sot_talos_balance.utils.sot_utils import XYZRPYToHomogeneous
from sot_talos_balance.meta_task_config import MetaTaskKineConfig
from dynamic_graph.sot.core.meta_tasks_kine import MetaTaskKine6d, MetaTaskKineCom, gotoNd
from dynamic_graph import plug
from dynamic_graph.sot.core import SOT
from dynamic_graph.sot.core.operator import Mix_of_vector

from dynamic_graph.ros import RosSubscribe
from dynamic_graph.sot.dynamics_pinocchio import DynamicPinocchio

dt = robot.timeStep;

# --- Subscriber
robot.subscriber = RosSubscribe("base_subscriber")
robot.subscriber.add("vector","position","/sot/base_link/position")
robot.subscriber.add("vector","velocity","/sot/base_link/velocity")

# --- Dynamic pinocchio
robot.rdynamic = DynamicPinocchio("real_dynamics")
robot.rdynamic.setModel(robot.dynamic.model)
robot.rdynamic.setData(robot.rdynamic.model.createData())

plug(robot.device.state,robot.rdynamic.position)
plug(robot.subscriber.position,robot.rdynamic.ffposition)

plug(robot.device.velocity,robot.rdynamic.velocity)
plug(robot.subscriber.velocity,robot.rdynamic.ffvelocity)

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
