from sot_talos_balance.create_entities_utils           import *
from sot_talos_balance.utils.plot_utils                import *
import sot_talos_balance.talos.parameter_server_conf   as param_server_conf
import sot_talos_balance.talos.base_estimator_conf     as base_estimator_conf
from dynamic_graph.sot.core.meta_tasks_kine            import MetaTaskKine6d, MetaTaskKineCom, gotoNd
from dynamic_graph.sot.core.matrix_util                import matrixToTuple
from dynamic_graph                                     import plug
from dynamic_graph.sot.core                            import SOT
from time                                              import sleep
from dynamic_graph.ros                                 import RosSubscribe
import os
import numpy                                           as np
from dynamic_graph.ros                                 import RosPublish


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

# --- SOT
robot.sot = SOT('sot')
robot.sot.setSize(robot.dynamic.getDimension())
plug(robot.sot.control,robot.device.control)
plug(robot.comTrajGen.x,    robot.taskCom.featureDes.errorIN);

robot.sot.push(robot.contactRF.task.name)
robot.sot.push(robot.taskCom.task.name)
robot.sot.push(robot.contactLF.task.name)
robot.device.control.recompute(0)

# --- ESTIMATION
robot.param_server            = create_parameter_server(param_server_conf,dt)
# robot.imu_offset_compensation = create_imu_offset_compensation(robot, dt)
robot.device_filters          = create_device_filters(robot, dt)
robot.imu_filters             = create_imu_filters(robot, dt)
robot.base_estimator          = create_base_estimator(robot, dt, base_estimator_conf) 
robot.be_filters              = create_be_filters(robot, dt)

robot_name='robot'
e2q = EulerToQuat('e2q')
plug(robot.base_estimator.q,e2q.euler)
robot.e2q = e2q
dcm_estimator = DcmEstimator('dcm_estimator')
dcm_estimator.init(dt, robot_name)
plug(robot.e2q.quaternion, dcm_estimator.q)
plug(robot.base_estimator.v, dcm_estimator.v)
robot.dcm_estimator = dcm_estimator

# --- ROS PUBLISHER
robot.publisher = create_rospublish(robot, 'robot_publisher')
create_topic(robot.publisher, robot.base_estimator, 'q', robot = robot, data_type='vector')
create_topic(robot.publisher, robot.base_estimator, 'q_imu', robot = robot, data_type='vector')
create_topic(robot.publisher, robot.device_filters.gyro_filter, 'x_filtered', robot = robot, data_type='vector')
create_topic(robot.publisher, robot.device_filters.acc_filter, 'x_filtered', robot = robot, data_type='vector')
create_topic(robot.publisher, robot.dcm_estimator, 'c', robot = robot, data_type='vector')
create_topic(robot.publisher, robot.dcm_estimator, 'dc', robot = robot, data_type='vector')
create_topic(robot.publisher, robot.base_estimator, 'v', robot = robot, data_type='vector')
create_topic(robot.publisher, robot.base_estimator, 'v_gyr', robot = robot, data_type='vector')

create_topic(robot.publisher, robot.dynamic, 'com', robot = robot, data_type='vector')

# --- ROS SUBSCRIBER
robot.subscriber = RosSubscribe("base_subscriber")
robot.subscriber.add("vector","position","/sot/base_link/position")
robot.subscriber.add("vector","velocity","/sot/base_link/velocity")
robot.subscriber.add("vector","q_est","/sot/base_estimator/q")
robot.subscriber.add("vector","q_imu","/sot/base_estimator/q_imu")
robot.subscriber.add("vector","gyro_f","/sot/gyro_filter/x_filtered")
robot.subscriber.add("vector","acc_f","/sot/acc_filter/x_filtered")
robot.subscriber.add("vector","com","/sot/dcm_estimator/c")
robot.subscriber.add("vector","vcom","/sot/dcm_estimator/dc")
robot.subscriber.add("vector","v_base","/sot/base_estimator/v")
robot.subscriber.add("vector","v_gyr","/sot/base_estimator/v_gyr")


robot.device.after.addSignal('{0}.position'.format(robot.subscriber.name)) # force recalculation
robot.device.after.addSignal('{0}.velocity'.format(robot.subscriber.name)) # force recalculation
robot.device.after.addSignal('{0}.com'.format(robot.subscriber.name)) # force recalculation
robot.device.after.addSignal('{0}.vcom'.format(robot.subscriber.name)) # force recalculation
robot.device.after.addSignal('{0}.v_base'.format(robot.subscriber.name)) # force recalculation

# robot.device.after.addSignal('{0}.velocity'.format(robot.subscriber.name)) # force recalculation
# robot.device.after.addSignal('{0}.q_est'.format(robot.subscriber.name))        # force recalculation
# robot.device.after.addSignal('{0}.q_imu'.format(robot.subscriber.name))        # force recalculation
# robot.device.after.addSignal('{0}.gyro'.format(robot.subscriber.name))        # force recalculation
# robot.device.after.addSignal('{0}.gyro_f'.format(robot.subscriber.name))        # force recalculation

