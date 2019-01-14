from sot_talos_balance.create_entities_utils import create_com_trajectory_generator
from dynamic_graph.sot.core.meta_tasks_kine import MetaTaskKine6d, MetaTaskKineCom, gotoNd
from dynamic_graph.sot.core.matrix_util import matrixToTuple
from dynamic_graph import plug
from dynamic_graph.sot.core import SOT
from numpy import eye
from time import sleep
import os

from dynamic_graph.ros import RosSubscribe
from dynamic_graph.sot.dynamics_pinocchio import DynamicPinocchio

from dynamic_graph.tracer_real_time import TracerRealTime
from sot_talos_balance.create_entities_utils import addTrace, dump_tracer
import matplotlib.pyplot as plt
import numpy as np

def main(robot):
    dt = robot.timeStep;
    robot.comTrajGen = create_com_trajectory_generator(dt,robot);

    robot.subscriber = RosSubscribe("base_subscriber")
    robot.subscriber.add("vector","position","/sot/base_link/position")
    robot.subscriber.add("vector","velocity","/sot/base_link/velocity")

    robot.rdynamic = DynamicPinocchio("real_dynamics")
    robot.rdynamic.setModel(robot.dynamic.model)
    robot.rdynamic.setData(robot.rdynamic.model.createData())

    plug(robot.device.robotState,robot.rdynamic.position)
    plug(robot.subscriber.position,robot.rdynamic.ffposition)

    plug(robot.device.robotVelocity,robot.rdynamic.velocity)
    plug(robot.subscriber.velocity,robot.rdynamic.ffvelocity)

    robot.rdynamic.acceleration.value = [0.0]*38

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
    robot.sot.push(robot.taskCom.task.name)
    robot.sot.push(robot.contactLF.task.name)
    robot.device.control.recompute(0)

    # --- TRACER
    robot.tracer = TracerRealTime("com_tracer")
    robot.tracer.setBufferSize(80*(2**20))
    robot.tracer.open('/tmp','dg_','.dat')
    robot.device.after.addSignal('{0}.triger'.format(robot.tracer.name))
    robot.device.after.addSignal('{0}.com'.format(robot.rdynamic.name))

    addTrace(robot.tracer, robot.comTrajGen, 'x')
    addTrace(robot.tracer, robot.dynamic, 'com')
    addTrace(robot.tracer, robot.rdynamic, 'com')

    robot.tracer.start()

    plug(robot.comTrajGen.x,    robot.taskCom.featureDes.errorIN);
    sleep(1.0);
    os.system("rosservice call \start_dynamic_graph")
    sleep(2.0);
    robot.comTrajGen.move(1,-0.025,4.0);
    sleep(5.0);
    robot.comTrajGen.startSinusoid(1,0.05,8.0);
    sleep(5.0);

    dump_tracer(robot.tracer)

	  # --- DISPLAY
    comDes_data = np.loadtxt('/tmp/dg_'+robot.comTrajGen.name+'-x.dat')
    comSot_data = np.loadtxt('/tmp/dg_'+robot.dynamic.name+'-com.dat')
    com_data    = np.loadtxt('/tmp/dg_'+robot.rdynamic.name+'-com.dat')
    

    plt.figure()
    plt.plot(com_data[:,1],'b-')
    plt.plot(comDes_data[:,1],'b--')
    plt.plot(comSot_data[:,1],'b:')
    plt.plot(com_data[:,2],'r-')
    plt.plot(comDes_data[:,2],'r--')
    plt.plot(comSot_data[:,2],'r:')
    plt.plot(com_data[:,3],'g-')
    plt.plot(comDes_data[:,3],'g--')
    plt.plot(comSot_data[:,3],'g:')
    plt.title('COM real vs desired vs SOT')
    plt.legend(['Real x','Desired x','SOT x','Real y','Desired y','SOT y','Real z','Desired z','SOT z'])

    plt.figure()
    plt.plot(com_data[:,1],'b-')
    plt.title('COM real x')
    plt.figure()
    plt.plot(comDes_data[:,1],'b--')
    plt.title('COM desired x')
    plt.figure()
    plt.plot(comSot_data[:,1],'b:')
    plt.title('COM SOT x')

    plt.figure()
    plt.plot(com_data[:,2],'r-')
    plt.title('COM real y')
    plt.figure()
    plt.plot(comDes_data[:,2],'r--')
    plt.title('COM desired y')
    plt.figure()
    plt.plot(comSot_data[:,2],'r:')
    plt.title('COM SOT y')

    plt.figure()
    plt.plot(com_data[:,3],'g-')
    plt.title('COM real z')
    plt.figure()
    plt.plot(comDes_data[:,3],'g--')
    plt.title('COM desired z')
    plt.figure()
    plt.plot(comSot_data[:,3],'g:')
    plt.title('COM SOT z')

    plt.show()

