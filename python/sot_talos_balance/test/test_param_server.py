#!/usr/bin/python
# -*- coding: utf-8 -*-1
from sot_talos_balance.create_entities_utils                  import *
from dynamic_graph                                            import plug
from time                                                     import sleep
import os
from sot_talos_balance.utils.sot_utils                        import Bunch
from dynamic_graph.sot.core.meta_tasks_kine                   import MetaTaskKine6d, MetaTaskKineCom, gotoNd
from dynamic_graph.sot.core.matrix_util                       import matrixToTuple
from dynamic_graph                                            import plug
import dynamic_graph                                          as dg
from dynamic_graph.sot.core                                   import SOT
from numpy                                                    import eye
from time                                                     import sleep

def get_default_conf():
    import sot_talos_balance.talos.balance_ctrl_conf as balance_ctrl_conf
    import sot_talos_balance.talos.admittance_ctrl_conf as admittance_ctrl_conf
    import sot_talos_balance.talos.base_estimator_conf as base_estimator_conf
    import sot_talos_balance.talos.control_manager_conf as control_manager_conf
    import sot_talos_balance.talos.force_torque_estimator_conf as force_torque_estimator_conf
    import sot_talos_balance.talos.joint_torque_controller_conf as joint_torque_controller_conf
    conf = Bunch();
    conf.balance_ctrl              = balance_ctrl_conf;
    conf.adm_ctrl                  = admittance_ctrl_conf;
    conf.base_estimator            = base_estimator_conf;
    conf.param_server              = control_manager_conf;
    conf.force_torque_estimator    = force_torque_estimator_conf;
    conf.joint_torque_controller   = joint_torque_controller_conf;
    return conf;

def main(robot, conf=None):
    if(conf is None):
        conf = get_default_conf();
    dt = robot.timeStep;
    NJ = robot.dimension-7    
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
    
    robot.sot.push(robot.contactRF.task.name)
    robot.sot.push(robot.taskCom.task.name)
    robot.sot.push(robot.contactLF.task.name)
    robot.device.control.recompute(0)
    
    #--- ENTITIES
    robot.param_server            = create_parameter_server(conf.param_server, dt)
    robot.example                 = create_example() # will output the number of joints of the robot thanks to a pointer to the RobotUtil created in parameter server
    
    # --- RUN SIMULATION
    plug(robot.comTrajGen.x,    robot.taskCom.featureDes.errorIN);
    sleep(1.0);
    os.system("rosservice call \start_dynamic_graph")
    sleep(2.0);
    robot.comTrajGen.move(1,-0.025,4.0);
    sleep(5.0);
    robot.comTrajGen.startSinusoid(1,0.05,8.0);
    sleep(1.0)
    robot.example.nbJoints.recompute(1)
    print(robot.example.nbJoints.value)

    
