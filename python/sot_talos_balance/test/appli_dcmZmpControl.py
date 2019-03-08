from sot_talos_balance.create_entities_utils import *
import sot_talos_balance.control_manager_conf          as param_server_conf
from dynamic_graph.sot.core.meta_tasks_kine import MetaTaskKine6d, MetaTaskKineCom, gotoNd
from dynamic_graph.sot.core.matrix_util import matrixToTuple
from dynamic_graph import plug
from dynamic_graph.sot.core import SOT
from math import sqrt

from dynamic_graph.tracer_real_time import TracerRealTime

robot.timeStep = robot.device.getTimeStep()
dt = robot.timeStep;

# -------------------------- DESIRED TRAJECTORY --------------------------

# --- Desired values
robot.dynamic.com.recompute(0)
comDes = robot.dynamic.com.value
dcmDes = comDes
zmpDes = comDes[:2] + (0.0,)
ddcomDes = (0.0,0.0,0.0)

# --- Pendulum parameters
mass = robot.dynamic.data.mass[0]
h = robot.dynamic.com.value[2]
g = 9.81
omega = sqrt(g/h)

# -------------------------- ESTIMATION --------------------------

# --- General Estimation
# robot.param_server = create_parameter_server(param_server_conf,dt)
# robot_name='robot'
# cdc_estimator = DcmEstimator('dcm_estimator')
# cdc_estimator.init(dt, robot_name)
# plug(robot.device.state, cdc_estimator.q)
# plug(robot.device.velocity, cdc_estimator.v)
# robot.cdc_estimator = cdc_estimator # cdc_estimator.c == robot.dynamic.com
robot.cdc_estimator = robot.dynamic

# --- DCM Estimation
estimator = DummyDcmEstimator("dummy")
estimator.omega.value = omega
# estimator.mass.value = 1.0
# plug(robot.cdc_estimator.c, estimator.com)
# plug(robot.cdc_estimator.dc,estimator.momenta)
estimator.mass.value = mass
plug(robot.cdc_estimator.com,estimator.com)
plug(robot.cdc_estimator.momenta,estimator.momenta)
estimator.init()
robot.estimator = estimator

# --- ZMP estimation
#zmp_estimator = SimpleZmpEstimator("zmpEst")
#robot.dynamic.createOpPoint('LF',robot.OperationalPointsMap['left-ankle'])
#robot.dynamic.createOpPoint('RF',robot.OperationalPointsMap['right-ankle'])
#plug(robot.dynamic.LF,zmp_estimator.poseLeft)
#plug(robot.dynamic.RF,zmp_estimator.poseRight)
## plug(robot.device_filters.ft_LF_filter.x_filtered,zmp_estimator.wrenchLeft)
## plug(robot.device_filters.ft_RF_filter.x_filtered,zmp_estimator.wrenchRight)
#plug(robot.device.forceLLEG,zmp_estimator.wrenchLeft)
#plug(robot.device.forceRLEG,zmp_estimator.wrenchRight)
#zmp_estimator.init()
#robot.zmp_estimator = zmp_estimator
robot.zmp_estimator = robot.dynamic

# -------------------------- ADMITTANCE CONTROL --------------------------

# --- DCM controller
Kp_dcm = [1.0,1.0,1.0]
Ki_dcm = [0.0,0.0,0.0] # zero (to be set later)
gamma_dcm = 0.2

dcm_controller = DcmController("dcmCtrl")

dcm_controller.Kp.value = Kp_dcm
dcm_controller.Ki.value = Ki_dcm
dcm_controller.decayFactor.value = gamma_dcm
dcm_controller.mass.value = mass
dcm_controller.omega.value = omega

plug(robot.cdc_estimator.com,dcm_controller.com)
plug(robot.estimator.dcm,dcm_controller.dcm)

dcm_controller.zmpDes.value = zmpDes # plug a signal here
dcm_controller.dcmDes.value = dcmDes # plug a signal here

dcm_controller.init(dt)

robot.dcm_control = dcm_controller

Ki_dcm = [0.1,0.1,0.0] # this value is employed later

# --- CoM admittance controller
Kp_adm = [0.0,0.0,0.0] # zero (to be set later)

com_admittance_control = ComAdmittanceController("comAdmCtrl")
com_admittance_control.Kp.value = Kp_adm
plug(robot.zmp_estimator.zmp,com_admittance_control.zmp)
com_admittance_control.zmpDes.value = zmpDes     # should be plugged to robot.dcm_control.zmpRef
com_admittance_control.ddcomDes.value = ddcomDes # plug a signal here

com_admittance_control.init(dt)
com_admittance_control.setState(comDes,[0.0,0.0,0.0])

robot.com_admittance_control = com_admittance_control

Kp_adm = [1.0,1.0,0.0] # this value is employed later

# -------------------------- SOT CONTROL --------------------------

# --- CONTACTS
#define contactLF and contactRF
robot.contactLF = MetaTaskKine6d('contactLF',robot.dynamic,'LF',robot.OperationalPointsMap['left-ankle'])
robot.contactLF.feature.frame('desired')
robot.contactLF.gain.setConstant(300)
robot.contactLF.keep()
locals()['contactLF'] = robot.contactLF

robot.contactRF = MetaTaskKine6d('contactRF',robot.dynamic,'RF',robot.OperationalPointsMap['right-ankle'])
robot.contactRF.feature.frame('desired')
robot.contactRF.gain.setConstant(300)
robot.contactRF.keep()
locals()['contactRF'] = robot.contactRF

# --- COM
robot.taskCom = MetaTaskKineCom(robot.dynamic)
plug(robot.com_admittance_control.comRef,robot.taskCom.featureDes.errorIN)
plug(robot.com_admittance_control.dcomRef,robot.taskCom.featureDes.errordotIN)
robot.taskCom.task.controlGain.value = 0
robot.taskCom.task.setWithDerivative(True)

# --- SOT solver
robot.sot = SOT('sot')
robot.sot.setSize(robot.dynamic.getDimension())
plug(robot.sot.control,robot.device.control)

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

# -------------------------- PLOTS --------------------------

# --- ROS PUBLISHER
robot.publisher = create_rospublish(robot, 'robot_publisher')        

rospub_signalName = '{0}_{1}'.format('fake', 'comDes')
topicname = '/sot/{0}/{1}'.format('fake', 'comDes')
robot.publisher.add('vector',rospub_signalName,topicname)
plug(robot.dcm_control.dcmDes, robot.publisher.signal(rospub_signalName))                                 # desired CoM (workaround)

rospub_signalName = '{0}_{1}'.format(robot.cdc_estimator.name, 'c')
topicname = '/sot/{0}/{1}'.format(robot.cdc_estimator.name, 'c')
robot.publisher.add('vector',rospub_signalName,topicname)
plug(robot.cdc_estimator.com, robot.publisher.signal(rospub_signalName))                                  # estimated CoM (workaround)
# create_topic(robot.publisher, robot.cdc_estimator, 'c', robot = robot, data_type='vector')                # estimated CoM (to be modified)

create_topic(robot.publisher, robot.com_admittance_control, 'comRef', robot = robot, data_type='vector')  # reference CoM
create_topic(robot.publisher, robot.dynamic, 'com', robot = robot, data_type='vector')                    # resulting SOT CoM

create_topic(robot.publisher, robot.dcm_control, 'dcmDes', robot = robot, data_type='vector')             # desired DCM
create_topic(robot.publisher, robot.estimator, 'dcm', robot = robot, data_type='vector')                  # estimated DCM

create_topic(robot.publisher, robot.dcm_control, 'zmpDes', robot = robot, data_type='vector')             # desired ZMP
create_topic(robot.publisher, robot.zmp_estimator, 'zmp', robot = robot, data_type='vector')              # estimated ZMP
create_topic(robot.publisher, robot.dcm_control, 'zmpRef', robot = robot, data_type='vector')             # resulting SOT CoM

# --- TRACER
robot.tracer = TracerRealTime("zmp_tracer")
robot.tracer.setBufferSize(80*(2**20))
robot.tracer.open('/tmp','dg_','.dat')
robot.device.after.addSignal('{0}.triger'.format(robot.tracer.name))

addTrace(robot.tracer, robot.dcm_control, 'dcmDes')             # desired CoM (workaround)
addTrace(robot.tracer, robot.dynamic, 'com')                    # estimated CoM (workaround)
# addTrace(robot.tracer, robot.cdc_estimator, 'com')              # estimated CoM (to be modified)
addTrace(robot.tracer, robot.com_admittance_control, 'comRef')  # reference CoM
# addTrace(robot.tracer, robot.dynamic, 'com')                    # resulting SOT CoM (already added)

# addTrace(robot.tracer, robot.dcm_control, 'dcmDes')           # desired DCM (already added)
addTrace(robot.tracer, robot.estimator, 'dcm')                  # estimated DCM

addTrace(robot.tracer, robot.dcm_control, 'zmpDes')             # desired ZMP
addTrace(robot.tracer, robot.zmp_estimator, 'zmp')              # estimated ZMP
addTrace(robot.tracer, robot.dcm_control, 'zmpRef')             # reference ZMP

# -------------------------- SIMULATION --------------------------

robot.tracer.start()

