from __future__ import print_function
from sot_talos_balance.example import Example

ex = Example("ciao")

print("Commands:")
print(ex.commands())

print("\nSignals (at creation):")
ex.displaySignals()

ex.firstAddend.value = 0.1
ex.secondAddend.value = 0.3

print("\nSignals (after pugging):")
ex.displaySignals()

ex.init()

ex.sum.recompute(1)

print( "\nInputs: %f, %f" % (ex.firstAddend.value, ex.secondAddend.value) )
print( "Output: %f" % ex.sum.value )

