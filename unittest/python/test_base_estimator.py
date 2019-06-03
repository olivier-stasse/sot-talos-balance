from sot_talos_balance.create_entities_utils                  import *
import sot_talos_balance.talos.parameter_server_conf          as param_server_conf

dt = 0.001
conf = Bunch()
robot_name = 'robot'

conf.param_server = param_server_conf
param_server = ParameterServer("param_server")     
param_server.init(dt, conf.param_server.urdfFileName, robot_name)
param_server.setJointsUrdfToSot(conf.param_server.urdftosot)
param_server.setRightFootForceSensorXYZ(conf.param_server.rightFootSensorXYZ)
param_server.setRightFootSoleXYZ(conf.param_server.rightFootSoleXYZ)
param_server.setImuJointName(conf.param_server.ImuJointName)

base_estimator = TalosBaseEstimator('base_estimator')
base_estimator.init(dt, robot_name)

example = Example('example')
example.firstAddend.value  = 0.
example.secondAddend.value = 0.
example.init(robot_name)  

