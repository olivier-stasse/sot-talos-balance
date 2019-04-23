import numpy as np

''' *********************** USER-PARAMETERS FOR FT CALIBRATION *********************** '''

handWeight = dict() # Force (N) applied by the hand when horizontal
handWeight['robot'] = [0., 0., -26.93]
handWeight['simu'] = [0., 0., -14.6048]