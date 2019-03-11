from sot_talos_balance.create_entities_utils import create_com_trajectory_generator
from sot_talos_balance.create_entities_utils import create_zmp_estimator
from dynamic_graph.sot.core.meta_tasks_kine import MetaTaskKine6d, MetaTaskKineCom, gotoNd
from dynamic_graph import plug
from dynamic_graph.sot.core import SOT

from dynamic_graph.tracer_real_time import TracerRealTime
from sot_talos_balance.create_entities_utils import addTrace, dump_tracer

robot.timeStep = robot.device.getTimeStep()
dt = robot.timeStep;
robot.comTrajGen = create_com_trajectory_generator(dt,robot);

# --- COM
robot.taskCom = MetaTaskKineCom(robot.dynamic)
robot.dynamic.com.recompute(0)
robot.taskCom.featureDes.errorIN.value = robot.dynamic.com.value
robot.taskCom.task.controlGain.value = 10

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

# -- estimator
# -- this NEEDS to be called AFTER the operational points LF and RF are created
robot.zmp_estimator = create_zmp_estimator(robot)

robot.sot = SOT('sot')
robot.sot.setSize(robot.dynamic.getDimension())
plug(robot.sot.control,robot.device.control)
plug(robot.comTrajGen.x,    robot.taskCom.featureDes.errorIN)

robot.sot.push(robot.contactRF.task.name)
robot.sot.push(robot.contactLF.task.name)
robot.sot.push(robot.taskCom.task.name)
robot.device.control.recompute(0)

# --- Fix robot.dynamic inputs
plug(robot.sot.control,robot.dynamic.velocity)
from dynamic_graph.sot.core import Derivator_of_Vector
robot.dvdt = Derivator_of_Vector("dv_dt")
robot.dvdt.dt.value = dt
plug(robot.sot.control,robot.dvdt.sin)
plug(robot.dvdt.sout,robot.dynamic.acceleration)

# --- TRACER
robot.tracer = TracerRealTime("zmp_tracer")
robot.tracer.setBufferSize(80*(2**20))
robot.tracer.open('/tmp','dg_','.dat')
robot.device.after.addSignal('{0}.triger'.format(robot.tracer.name))
robot.device.after.addSignal('{0}.zmp'.format(robot.zmp_estimator.name))
robot.device.after.addSignal('{0}.zmp'.format(robot.dynamic.name))

addTrace(robot.tracer, robot.zmp_estimator, 'zmp')
addTrace(robot.tracer, robot.dynamic, 'com')
addTrace(robot.tracer, robot.dynamic, 'zmp')
addTrace(robot.tracer, robot.device, 'forceLLEG')
addTrace(robot.tracer, robot.device, 'forceRLEG')

robot.tracer.start()

