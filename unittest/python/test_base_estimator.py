from sot_talos_balance.create_entities_utils                  import *
from IPython                                                  import embed
from sot_talos_balance.example                                import Example
from sot_talos_balance.parameter_server                       import ParameterServer
from sot_talos_balance.base_estimator                         import BaseEstimator
from sot_talos_balance.utils.sot_utils                        import Bunch
from sot_talos_balance.example                                import Example
import sot_talos_balance.talos.control_manager_conf           as param_server_conf

dt = 0.001
conf = Bunch()
robot_name = 'robot'

conf.param_server = param_server_conf
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

embed()







