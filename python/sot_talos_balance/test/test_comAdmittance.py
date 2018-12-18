from sot_talos_balance.create_entities_utils import create_com_admittance_controller, create_dummy_dcm_estimator
from dynamic_graph.sot.core.meta_tasks_kine import MetaTaskKine6d, MetaTaskKineCom, gotoNd
from dynamic_graph.sot.core.matrix_util import matrixToTuple
from dynamic_graph import plug
from dynamic_graph.sot.core import SOT
from numpy import eye
from time import sleep
import sys
import os

def main(robot):
    dt = robot.timeStep;

    # --- COM
    robot.taskCom = MetaTaskKineCom(robot.dynamic)
    robot.dynamic.com.recompute(0)
    robot.taskCom.featureDes.errorIN.value = robot.dynamic.com.value
    robot.taskCom.task.controlGain.value = 10

    robot.estimator = create_dummy_dcm_estimator(robot)

    # --- Admittance controller
    Kp = [0.0,0.0,0.0]
    robot.com_admittance_control = create_com_admittance_controller(Kp,dt,robot)

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

    robot.sot = SOT('sot')
    robot.sot.setSize(robot.dynamic.getDimension())
    plug(robot.sot.control,robot.device.control)

    robot.sot.push(robot.contactRF.task.name)
    robot.sot.push(robot.contactLF.task.name)
    robot.sot.push(robot.taskCom.task.name)
    robot.device.control.recompute(0)

    plug(robot.com_admittance_control.comRef,robot.taskCom.featureDes.errorIN)
    sleep(1.0)
    os.system("rosservice call \start_dynamic_graph")
    sleep(2.0)

    plug(robot.estimator.dcm,robot.com_admittance_control.zmpDes)

    print("Activating controller...")
    robot.com_admittance_control.setState(robot.dynamic.com.value,[0.0,0.0,0.0])
    robot.com_admittance_control.Kp.value = [10.0,10.0,0.0]

    sleep(5.0)
