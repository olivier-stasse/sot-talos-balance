from __future__ import print_function
from sot_talos_balance.joint_position_controller import JointPositionController

controller = JointPositionController("ciao")

print("Commands:")
print(controller.commands())

print("\nSignals (at creation):")
controller.displaySignals()

controller.q.value = (0.0,0.0)
controller.qDes.value = (1.0,1.0)
controller.dqDes.value = (0.0,0.0)

Kp = (10.0,10.0)

controller.init(Kp)

controller.dqRef.recompute(1)

print( "\nKp: %s" % (Kp,) )

print( "\nq: %s" % (controller.q.value,) )
print( "qDes: %s" % (controller.qDes.value,) )
print( "dqDes: %s" % (controller.dqDes.value,) )

print( "\ndqRef: %s" % (controller.dqRef.value,) )

#print( "\nInputs: %f, %f" % (ex.firstAddend.value, ex.secondAddend.value) )
#print( "Output: %f" % ex.sum.value )

