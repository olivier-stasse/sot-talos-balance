from __future__ import print_function

from sot_talos_balance.foot_force_difference_controller import FootForceDifferenceController
import numpy as np
import pinocchio as pin
from numpy.testing import assert_almost_equal as assertApprox

# --- General ---
print("--- General ---")

controller = FootForceDifferenceController("footController")
controller.init()

controller.dfzAdmittance.value = 0.

controller.wrenchRight.value = [0.]*6
controller.wrenchLeft.value = [0.]*6
controller.wrenchRightDes.value = [0.]*6
controller.wrenchLeftDes.value = [0.]*6

controller.vdcFrequency.value = 0.
controller.vdcDamping.value = 0.

controller.posRightDes.value = np.eye(4).tolist()
controller.posLeftDes.value = np.eye(4).tolist()
controller.posRight.value = np.eye(4).tolist()
controller.posLeft.value = np.eye(4).tolist()

controller.vRight.recompute(0)
controller.vLeft.recompute(0)

print(controller.vRight.value)
assertApprox([0.0]*6,controller.vRight.value)
print(controller.vLeft.value)
assertApprox([0.0]*6,controller.vLeft.value)

