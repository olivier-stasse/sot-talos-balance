'''Test foot sinusoid control'''

from time import sleep
from sys import argv

from sot_talos_balance.utils.run_test_utils import run_test, run_ft_calibration, runCommandClient, ask_for_confirmation

try:
    # Python 2
    input = raw_input  # noqa
except NameError:
    pass

runCommandClient('test_folder = None')
run_test('appli_dcm_zmp_control.py')

run_ft_calibration('robot.ftc')

input("Wait before running the test")

# Connect ZMP reference and reset controllers
print('Connect ZMP reference')
runCommandClient('plug(robot.zmp_estimator.emergencyStop,robot.cm.emergencyStop_zmp)')
runCommandClient('plug(robot.dcm_control.zmpRef,robot.com_admittance_control.zmpDes)')
runCommandClient('robot.com_admittance_control.setState(robot.wp.comDes.value,[0.0,0.0,0.0])')
runCommandClient('robot.com_admittance_control.Kp.value = Kp_adm')
runCommandClient('robot.dcm_control.resetDcmIntegralError()')
runCommandClient('robot.dcm_control.Ki.value = Ki_dcm')
runCommandClient('robot.dcm_control.Kz.value = Kz_dcm')


c = ask_for_confirmation("Raise the foot?")
if c:
    print("Putting the robot in position...")
    runCommandClient('robot.comTrajGen.move(1,-0.07,10.0)')
    sleep(10.0)
    print("Robot is in position!")

    foot_on_ground = True

    c2 = ask_for_confirmation("Confirm raising the foot?")
    if c2:
        print("Raising the foot...")
        runCommandClient('h = robot.dynamic.LF.value[2][3]')
        runCommandClient('robot.lfTrajGen.move(2,h+0.05,10.0)')
        sleep(10.0)
        print("Foot has been raised!")
        foot_on_ground = False
        c3 = ask_for_confirmation("Move to sinusoid pose?")
        if c3:
            print("Go to sinusoid pose...")
            runCommandClient('robot.lfTrajGen.move(0,-0.1,10.0)')
            runCommandClient('robot.lfTrajGen.move(2,h+0.1,10.0)')
            sleep(10.0)
            print("The foot is in position!")
            c6 = ask_for_confirmation("Start sinusoid move?")
            if c6:
                print("Start sinusoid move...")
                runCommandClient('robot.lfTrajGen.startSinusoid(0,0.1,6.0)')
                runCommandClient('robot.lfTrajGen.startSinusoid(2,h+0.05,3.0)')
                print("Sinusoid started!")
            else:
                print("Not executing the sinusoid")
        else:
            print("Not Moving to sinusoid")
        c4 = ask_for_confirmation("Put the foot back?")
        if c4:
            print("Stopping the robot...")
            runCommandClient('robot.lfTrajGen.stop(0)')
            sleep(5.0)
            runCommandClient('robot.lfTrajGen.stop(2)')  
            sleep(5.0)   
            print("Putting the robot back...")       
            runCommandClient('robot.lfTrajGen.move(0,0,10.0)')
            sleep(15.0)
            runCommandClient('robot.lfTrajGen.move(2,h,10.0)')
            sleep(15.0)
            print("The foot is back in position!")
            foot_on_ground = True
        else:
            print("Not putting the foot back")
    else:
        print("Not raising the foot")

    if foot_on_ground:
        c5 = ask_for_confirmation("Put the robot back?")
        if c5:
            print("Putting the robot back...")
            runCommandClient('robot.comTrajGen.move(1,0.0,10.0)')
            sleep(10.0)
            print("The robot is back in position!")
        else:
            print("Not putting the robot back")
else:
    print("Not raising the foot")

print('Bye!')

