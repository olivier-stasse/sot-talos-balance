from sot_talos_balance.create_entities_utils import *
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

# --- Merger
robot.statemix = Mix_of_vector("statemix")

robot.statemix.setSignalNumber(3)

robot.statemix.addSelec(1,0,6)
robot.statemix.addSelec(2,6,robot.dynamic.getDimension()-6)

robot.statemix.default.value=[0.0]*robot.dynamic.getDimension()
plug(robot.subscriber.position,robot.statemix.signal("sin1"))
plug(robot.device.joint_angles,robot.statemix.signal("sin2"))

# --- Dynamic pinocchio
robot.rdynamic = DynamicPinocchio("real_dynamics")
robot.rdynamic.setModel(robot.dynamic.model)
robot.rdynamic.setData(robot.rdynamic.model.createData())

plug(robot.device.robotState,robot.rdynamic.position)
plug(robot.subscriber.position,robot.rdynamic.ffposition)

plug(robot.device.robotVelocity,robot.rdynamic.velocity)
plug(robot.subscriber.velocity,robot.rdynamic.ffvelocity)

robot.rdynamic.acceleration.value = [0.0]*robot.dynamic.getDimension()

# --- Posture
robot.taskPos = MetaTaskKineConfig(robot.rdynamic)
robot.taskPos.featureDes.errorIN.value = robot.dynamic.position.value
robot.taskPos.task.controlGain.value = 10
plug(robot.statemix.sout,robot.taskPos.feature.errorIN)

# --- SOLVER
robot.sot = SOT('sot')
robot.sot.setSize(robot.dynamic.getDimension())
robot.sot.push(robot.taskPos.task.name)

robot.device.control.value = [0.0]*robot.dynamic.getDimension()

# --- TRACER
robot.tracer = TracerRealTime("com_tracer")
robot.tracer.setBufferSize(80*(2**20))
robot.tracer.open('/tmp','dg_','.dat')
robot.device.after.addSignal('{0}.triger'.format(robot.tracer.name))
robot.device.after.addSignal('{0}.state'.format(robot.device.name))
robot.device.after.addSignal('{0}.sout'.format(robot.statemix.name))
robot.device.after.addSignal('{0}.errorIN'.format(robot.taskPos.featureDes.name))

addTrace(robot.tracer, robot.taskPos.featureDes, 'errorIN')
addTrace(robot.tracer, robot.device, 'state')
addTrace(robot.tracer, robot.statemix, 'sout')

robot.tracer.start()
