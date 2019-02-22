#! /usr/bin/python

# This file is meant as a temporary replacement for the script plot_zmp.sh while rqt_plot is not working
# It should be removed once the problem with rqt_plot is solved
# Usage: plot_zmp.py [time in seconds (default: 5)]

from sot_talos_balance.utils.run_test_utils import runCommandClient, evalCommandClient
from sot_talos_balance.utils.plot_utils     import *
from sys import argv

try:
    time = float(argv[1])
except:
    time = 5.0

runCommandClient("from sot_talos_balance.utils.plot_utils import *")
runCommandClient("dump_sot_sigs(robot,[[robot.zmp_estimator, 'zmp'],[robot.dcm_control, 'zmpRef'],[robot.estimator, 'dcm'],[robot.dynamic, 'com'],[robot.com_admittance_control, 'comRef']],"+str(time)+")")

dcm_data    = np.loadtxt('/tmp/dg_' + evalCommandClient('robot.estimator.name') + '-dcm.dat')
zmp_data    = np.loadtxt('/tmp/dg_' + evalCommandClient('robot.zmp_estimator.name') + '-zmp.dat')
zmpDes_data = np.loadtxt('/tmp/dg_' + evalCommandClient('robot.dcm_control.name') + '-zmpRef.dat')
com_data    = np.loadtxt('/tmp/dg_' + evalCommandClient('robot.dynamic.name') + '-com.dat')
comDes_data = np.loadtxt('/tmp/dg_' + evalCommandClient('robot.com_admittance_control.name') + '-comRef.dat')

plt.ion()

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

raw_input("Wait before leaving the simulation")

