'''Test CoM admittance control as described in paper.'''
from sot_talos_balance.create_entities_utils import create_com_admittance_controller, create_dummy_dcm_estimator, create_dcm_controller
from dynamic_graph.sot.core.meta_tasks_kine import MetaTaskKine6d, MetaTaskKineCom, gotoNd
from dynamic_graph.sot.core.matrix_util import matrixToTuple
from dynamic_graph import plug
from dynamic_graph.sot.core import SOT
from numpy import eye
from time import sleep
import sys
import os

from dynamic_graph.tracer_real_time import TracerRealTime
from sot_talos_balance.create_entities_utils import addTrace, dump_tracer
import matplotlib.pyplot as plt
import numpy as np

from sot_talos_balance.utils.gazebo_utils import apply_force

def main(robot):
    dt = robot.timeStep;

    # --- COM
    robot.taskCom = MetaTaskKineCom(robot.dynamic)
    robot.dynamic.com.recompute(0)
    robot.taskCom.featureDes.errorIN.value = robot.dynamic.com.value
    robot.taskCom.task.controlGain.value = 0
    robot.taskCom.task.setWithDerivative(True)

    # --- Dummy estimator
    robot.estimator = create_dummy_dcm_estimator(robot)

    # --- DCM controller
    Kp_dcm = [500.0,500.0,500.0]
    Ki_dcm = [0.0,0.0,0.0]
    robot.dcm_control = create_dcm_controller(Kp_dcm,Ki_dcm,dt,robot,robot.estimator.dcm)

    # --- Admittance controller
    Kp_adm = [0.0,0.0,0.0]
    robot.com_admittance_control = create_com_admittance_controller(Kp_adm,dt,robot)

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

    robot.sot = SOT('sot')
    robot.sot.setSize(robot.dynamic.getDimension())
    plug(robot.sot.control,robot.device.control)

    robot.sot.push(robot.contactRF.task.name)
    robot.sot.push(robot.contactLF.task.name)
    robot.sot.push(robot.taskCom.task.name)
    robot.device.control.recompute(0)

    # --- TRACER
    robot.tracer = TracerRealTime("zmp_tracer")
    robot.tracer.setBufferSize(80*(2**20))
    robot.tracer.open('/tmp','dg_','.dat')
    robot.device.after.addSignal('{0}.triger'.format(robot.tracer.name))

    addTrace(robot.tracer, robot.dynamic, 'zmp')
    addTrace(robot.tracer, robot.dcm_control, 'zmpRef')
    addTrace(robot.tracer, robot.estimator, 'dcm')
    addTrace(robot.tracer, robot.dynamic, 'com')
    addTrace(robot.tracer, robot.com_admittance_control, 'comRef')

    # SIMULATION

    # begin with constant references
    plug(robot.com_admittance_control.comRef,robot.taskCom.featureDes.errorIN)
    plug(robot.com_admittance_control.dcomRef,robot.taskCom.featureDes.errordotIN)
    sleep(1.0)
    os.system("rosservice call \start_dynamic_graph")
    sleep(2.0)

    # connect ZMP control signal and reset controllers
    plug(robot.dcm_control.zmpRef,robot.com_admittance_control.zmpDes)

    robot.com_admittance_control.setState(robot.dynamic.com.value,[0.0,0.0,0.0])
    robot.com_admittance_control.Kp.value = [10.0,10.0,0.0]
    robot.dcm_control.resetDcmIntegralError()
    robot.dcm_control.Ki.value = [1.0,1.0,0.0]

    robot.tracer.start()

    sleep(5.0)

    # kick the robot on the chest to test its stability
    apply_force([-1000.0,0,0],0.01)

    sleep(5.0)

    dump_tracer(robot.tracer)

	  # --- DISPLAY
    dcm_data = np.loadtxt('/tmp/dg_'+robot.estimator.name+'-dcm.dat')
    zmp_data = np.loadtxt('/tmp/dg_'+robot.dynamic.name+'-zmp.dat')
    zmpDes_data = np.loadtxt('/tmp/dg_'+robot.dcm_control.name+'-zmpRef.dat')
    com_data = np.loadtxt('/tmp/dg_'+robot.dynamic.name+'-com.dat')
    comDes_data = np.loadtxt('/tmp/dg_'+robot.com_admittance_control.name+'-comRef.dat')

    plt.figure()
    plt.plot(dcm_data[:,1],'b-')
    plt.plot(dcm_data[:,2],'r-')
    plt.title('DCM')
    plt.legend(['x','y'])

    plt.figure()
    plt.plot(com_data[:,1],'b-')
    plt.plot(comDes_data[:,1],'b--')
    plt.plot(com_data[:,2],'r-')
    plt.plot(comDes_data[:,2],'r--')
    plt.plot(com_data[:,3],'g-')
    plt.plot(comDes_data[:,3],'g--')
    plt.title('COM real vs desired')
    plt.legend(['Real x','Desired x','Real y','Desired y','Real z','Desired z'])

    plt.figure()
    plt.plot(zmp_data[:,1],'b-')
    plt.plot(zmpDes_data[:,1],'b--')
    plt.plot(zmp_data[:,2],'r-')
    plt.plot(zmpDes_data[:,2],'r--')
    plt.title('ZMP real vs desired')
    plt.legend(['Real x','Desired x','Real y','Desired y'])

    plt.figure()
    plt.plot(zmp_data[:,1] - zmpDes_data[:,1],'b-')
    plt.plot(zmp_data[:,2] - zmpDes_data[:,2],'r-')
    plt.title('ZMP error')
    plt.legend(['Error on x','Error on y'])

    plt.show()

