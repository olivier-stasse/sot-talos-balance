from __future__ import print_function
from sot_talos_balance.admittance_controller_single_joint import AdmittanceControllerSingleJoint

controller = AdmittanceControllerSingleJoint("ciao")

print("Commands:")
print(controller.commands())

print("\nSignals (at creation):")
controller.displaySignals()

N_JOINTS = 2

controller.Kp.value = tuple(N_JOINTS*[10.0])
controller.state.value = tuple([0.0]*6 + N_JOINTS*[0.0])
controller.tauDes.value = tuple(N_JOINTS*[0.0])
controller.tau.value = tuple(N_JOINTS*[0.0])

print( "\nKp:   %s" % (controller.Kp.value,) )
print( "\nq:    %s" % (controller.state.value,) )
print( "tauDes: %s" % (controller.tauDes.value,) )
print( "tau:    %s" % (controller.tau.value,) )

q = N_JOINTS*[1.0]
dt = 1

controller.init(dt,N_JOINTS)

controller.setPosition(q)
controller.qRef.recompute(0)

print( "\nqRef: %s" % (controller.qRef.value,) )

controller.tauDes.value = tuple(N_JOINTS*[1.0])

controller.qRef.recompute(1)

print( "\nqRef: %s" % (controller.qRef.value,) )
