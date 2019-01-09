from __future__ import print_function
from sot_talos_balance.simple_zmp_estimator import SimpleZmpEstimator
from numpy.testing import assert_almost_equal as assertApprox

estimator = SimpleZmpEstimator("ciao")

print("\nSignals (at creation):")
estimator.displaySignals()

estimator.wrenchLeft.value  = [0.0,0.0,10.0,0.0,0.0,0.0]
estimator.wrenchRight.value = [0.0,0.0,10.0,0.0,0.0,0.0]

estimator.poseLeft.value  = [ [1.0, 0.0, 0.0, 1.0],
                              [0.0, 1.0, 0.0, 0.0],
                              [0.0, 0.0, 1.0, 0.1],
                              [0.0, 0.0, 0.0, 1.0]
                            ]
estimator.poseRight.value = [ [1.0, 0.0, 0.0, 0.0],
                              [0.0, 1.0, 0.0, 1.0],
                              [0.0, 0.0, 1.0, 0.1],
                              [0.0, 0.0, 0.0, 1.0]
                            ]

print()
print( "wrenchLeft:  %s" % (estimator.wrenchLeft.value,) )
print( "wrenchRight: %s" % (estimator.wrenchRight.value,) )
print( "poseLeft:  %s" % (estimator.poseLeft.value,) )
print( "poseRight: %s" % (estimator.poseRight.value,) )

estimator.init()

estimator.zmp.recompute(0)

copLeft  = (1.0, 0.0, 0.0)
copRight = (0.0, 1.0, 0.0)
zmp = (0.5, 0.5, 0.0)

print()
print( "copLeft:  %s" % (estimator.copLeft.value,) )
assertApprox(estimator.copLeft.value, copLeft)
print( "copRight: %s" % (estimator.copRight.value,) )
assertApprox(estimator.copRight.value, copRight)
print( "zmp:      %s" % (estimator.zmp.value,) )
assertApprox(estimator.zmp.value, zmp)

