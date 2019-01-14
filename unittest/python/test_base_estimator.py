from sot_talos_balance.create_entities_utils import *
from sot_talos_balance.utils.plot_utils import *
import sot_talos_balance.talos_conf as conf
import sot_talos_balance.motor_parameters as motor_params
import sot_talos_balance.control_manager_conf as control_manager_conf
from dynamic_graph.sot.core.meta_tasks_kine import MetaTaskKine6d, MetaTaskKineCom, gotoNd
from dynamic_graph.sot.core.matrix_util import matrixToTuple
from dynamic_graph import plug
import dynamic_graph as dg
from dynamic_graph.sot.core import SOT
from numpy import eye
from time import sleep
import os
from IPython import embed
from sot_talos_balance.control_manager                        import ControlManager
from sot_talos_balance.example                                import Example
from sot_talos_balance.parameter_server                       import ParameterServer
from dynamic_graph.tracer_real_time                           import TracerRealTime
from time                                                     import sleep
from sot_talos_balance.base_estimator                         import BaseEstimator
from sot_talos_balance.madgwickahrs                           import MadgwickAHRS
from sot_talos_balance.imu_offset_compensation                import ImuOffsetCompensation
from sot_talos_balance.utils.sot_utils                        import Bunch
from sot_talos_balance.example                                import Example
import sot_talos_balance.talos.control_manager_conf as control_manager_conf

dt = 0.001
conf = Bunch()
robot_name = 'robot'

conf.param_server = control_manager_conf
param_server = ParameterServer("param_server")     
param_server.init(dt, conf.param_server.urdfFileName, robot_name)
param_server.setRightFootForceSensorXYZ(conf.param_server.rightFootSensorXYZ)
param_server.setRightFootSoleXYZ(conf.param_server.rightFootSoleXYZ)

base_estimator = BaseEstimator('base_estimator')
base_estimator.init(dt, robot_name)

example = Example('example')
example.firstAddend.value  = 0.
example.secondAddend.value = 0.
example.init(robot_name)  








