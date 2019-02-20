from sot_talos_balance.create_entities_utils import *
from sot_talos_balance.meta_task_config import MetaTaskKineConfig
from dynamic_graph.sot.core.meta_tasks_kine import MetaTaskKine6d, MetaTaskKineCom, gotoNd
from dynamic_graph import plug
from dynamic_graph.sot.core import SOT
from dynamic_graph.sot.core.operator import Mix_of_vector, Selec_of_vector

from dynamic_graph.ros import RosSubscribe
from dynamic_graph.sot.dynamics_pinocchio import DynamicPinocchio

robot.timeStep = robot.device.getTimeStep()
dt = robot.timeStep;

# --- COM trajectory generator
robot.comTrajGen = create_com_trajectory_generator(dt,robot);

# --- Subscriber
robot.subscriber = RosSubscribe("base_subscriber")
robot.subscriber.add("vector","position","/sot/base_link/position")
robot.subscriber.add("vector","velocity","/sot/base_link/velocity")

# --- Merger
robot.stateselec = Selec_of_vector("stateselec")
robot.stateselec.selec(6,robot.dynamic.getDimension())
plug(robot.device.robotState,robot.stateselec.sin)

robot.stateselec = Selec_of_vector("stateselec")
robot.stateselec.selec(6,robot.dynamic.getDimension())
plug(robot.device.robotState,robot.stateselec.sin)

robot.statemix = Mix_of_vector("statemix")

robot.statemix.setSignalNumber(3)

robot.statemix.addSelec(1,0,6)
robot.statemix.addSelec(2,6,robot.dynamic.getDimension()-6)

robot.statemix.default.value=[0.0]*robot.dynamic.getDimension()
plug(robot.subscriber.position,robot.statemix.signal("sin1"))
plug(robot.stateselec.sout,robot.statemix.signal("sin2"))

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

# --- COM
robot.taskCom = MetaTaskKineCom(robot.rdynamic)
robot.dynamic.com.recompute(0)
robot.taskCom.featureDes.errorIN.value = robot.dynamic.com.value
robot.taskCom.task.controlGain.value = 10
plug(robot.comTrajGen.x, robot.taskCom.featureDes.errorIN)

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
robot.sot.push(robot.taskPos.task.name)

robot.device.control.value = [0.0]*robot.dynamic.getDimension()
