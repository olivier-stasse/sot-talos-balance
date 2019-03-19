from sot_talos_balance.create_entities_utils import *
from dynamic_graph.sot.core.meta_tasks_kine import MetaTaskKine6d, MetaTaskKineCom, gotoNd
from dynamic_graph import plug
from dynamic_graph.sot.core import SOT

from dynamic_graph.tracer_real_time import TracerRealTime

robot.timeStep = robot.device.getTimeStep()
dt = robot.timeStep;

# -------------------------- DESIRED TRAJECTORY --------------------------

robot.comTrajGen = create_com_trajectory_generator(dt,robot);

# -------------------------- ESTIMATION --------------------------

# --- filters
filters = Bunch();    
filters.ft_RF_filter  = create_butter_lp_filter_Wn_04_N_2("ft_RF_filter", dt, 6)
filters.ft_LF_filter  = create_butter_lp_filter_Wn_04_N_2("ft_LF_filter", dt, 6)
plug(robot.device.forceRLEG, filters.ft_RF_filter.x)
plug(robot.device.forceLLEG, filters.ft_LF_filter.x)
robot.device_filters = filters

# --- ZMP estimation (disconnected)
zmp_estimator = SimpleZmpEstimator("zmpEst")
robot.dynamic.createOpPoint('sole_LF','left_sole_link')
robot.dynamic.createOpPoint('sole_RF','right_sole_link')
plug(robot.dynamic.sole_LF,zmp_estimator.poseLeft)
plug(robot.dynamic.sole_RF,zmp_estimator.poseRight)
plug(robot.device_filters.ft_LF_filter.x_filtered,zmp_estimator.wrenchLeft)
plug(robot.device_filters.ft_RF_filter.x_filtered,zmp_estimator.wrenchRight)
#plug(robot.device.forceLLEG,zmp_estimator.wrenchLeft)
#plug(robot.device.forceRLEG,zmp_estimator.wrenchRight)
zmp_estimator.init()
robot.zmp_estimator = zmp_estimator

# -------------------------- SOT CONTROL --------------------------

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

# --- COM
robot.taskCom = MetaTaskKineCom(robot.dynamic)
robot.dynamic.com.recompute(0)
robot.taskCom.featureDes.errorIN.value = robot.dynamic.com.value
robot.taskCom.task.controlGain.value = 10

# --- SOT solver
robot.sot = SOT('sot')
robot.sot.setSize(robot.dynamic.getDimension())
plug(robot.sot.control,robot.device.control)
plug(robot.comTrajGen.x,    robot.taskCom.featureDes.errorIN)

robot.sot.push(robot.contactRF.task.name)
robot.sot.push(robot.contactLF.task.name)
robot.sot.push(robot.taskCom.task.name)
robot.device.control.recompute(0)

# --- Fix robot.dynamic inputs
plug(robot.device.velocity,robot.dynamic.velocity)
from dynamic_graph.sot.core import Derivator_of_Vector
robot.dvdt = Derivator_of_Vector("dv_dt")
robot.dvdt.dt.value = dt
plug(robot.device.velocity,robot.dvdt.sin)
plug(robot.dvdt.sout,robot.dynamic.acceleration)

# -------------------------- PLOTS --------------------------

# --- ROS PUBLISHER
robot.publisher = create_rospublish(robot, 'robot_publisher')        

create_topic(robot.publisher, robot.zmp_estimator, 'zmp', robot = robot, data_type='vector')              # estimated ZMP
create_topic(robot.publisher, robot.zmp_estimator, 'emergencyStop', robot = robot, data_type='boolean')   # ZMP emergency stop
create_topic(robot.publisher, robot.dynamic, 'com', robot = robot, data_type='vector')                    # SOT CoM
create_topic(robot.publisher, robot.dynamic, 'zmp', robot = robot, data_type='vector')                    # SOT ZMP
create_topic(robot.publisher, robot.device, 'forceLLEG', robot = robot, data_type='vector')               # force on left foot
create_topic(robot.publisher, robot.device, 'forceRLEG', robot = robot, data_type='vector')               # force on right foot

# --- TRACER
robot.tracer = TracerRealTime("zmp_tracer")
robot.tracer.setBufferSize(80*(2**20))
robot.tracer.open('/tmp','dg_','.dat')
robot.device.after.addSignal('{0}.triger'.format(robot.tracer.name))

addTrace(robot.tracer, robot.zmp_estimator, 'zmp')
addTrace(robot.tracer, robot.dynamic, 'com')
addTrace(robot.tracer, robot.dynamic, 'zmp')
addTrace(robot.tracer, robot.device, 'forceLLEG')
addTrace(robot.tracer, robot.device, 'forceRLEG')

robot.tracer.start()

