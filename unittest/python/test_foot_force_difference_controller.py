from __future__ import print_function

from sot_talos_balance.foot_force_difference_controller import FootForceDifferenceController
import numpy as np
import pinocchio as pin
from numpy.testing import assert_almost_equal as assertApprox

controller = FootForceDifferenceController("footController")
controller.init()

controller.dfzAdmittance.value = 1.

controller.wrenchRight.value = [0.]*2 + [500.] + [0.]*3
controller.wrenchLeft.value  = [0.]*2 + [300.] + [0.]*3
controller.wrenchRightDes.value = [0.]*2 + [400.] + [0.]*3
controller.wrenchLeftDes.value  = [0.]*2 + [400.] + [0.]*3

print("wrenchRight:    %s" % str(controller.wrenchRight.value))
print("wrenchLeft:     %s" % str(controller.wrenchLeft.value))
print("wrenchRightDes: %s" % str(controller.wrenchRightDes.value))
print("wrenchLeftDes:  %s" % str(controller.wrenchLeftDes.value))
print()

controller.vdcFrequency.value = 0.
controller.vdcDamping.value = 0.

controller.posRightDes.value = np.eye(4).tolist()
controller.posLeftDes.value = np.eye(4).tolist()
controller.posRight.value = np.eye(4).tolist()
controller.posLeft.value = np.eye(4).tolist()

controller.vRight.recompute(0)
controller.vLeft.recompute(0)

# There is more pressure on the right foot.
# Therefore, the right foot must go up to reduce it
vRight = [0.]*2 + [ 100.] + [0.]*3
vLeft  = [0.]*2 + [-100.] + [0.]*3

print("Expected vRight: %s" % str(vRight))
print("Actual vRight:   %s" % str(controller.vRight.value))
print("Expected vLeft:  %s" % str(vLeft))
print("Actual vLeft:    %s" % str(controller.vLeft.value))

assertApprox(vRight,controller.vRight.value)
assertApprox(vLeft,controller.vLeft.value)

