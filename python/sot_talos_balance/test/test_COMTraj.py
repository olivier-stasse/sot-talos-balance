from sot_talos_balance.create_entities_utils import create_com_trajectory_generator
from dynamic_graph.sot.core.meta_tasks_kine import MetaTaskKine6d, MetaTaskKineCom, gotoNd
from dynamic_graph.sot.core.matrix_util import matrixToTuple
from dynamic_graph import plug
from dynamic_graph.sot.core import SOT
from numpy import eye
from time import sleep
import os

def main(robot):
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

    robot.sot = SOT('sot')
    robot.sot.setSize(robot.dynamic.getDimension())
    plug(robot.sot.control,robot.device.control)

    robot.sot.push(robot.contactRF.task.name)
    robot.sot.push(robot.contactLF.task.name)
    robot.sot.push(robot.taskCom.task.name)
    robot.device.control.recompute(0)

    plug(robot.comTrajGen.x,    robot.taskCom.featureDes.errorIN);
    sleep(1.0);
    os.system("rosservice call \start_dynamic_graph")
    sleep(2.0);
    robot.comTrajGen.move(1,-0.025,4.0);
    sleep(5.0);
    robot.comTrajGen.startSinusoid(1,0.05,8.0);
