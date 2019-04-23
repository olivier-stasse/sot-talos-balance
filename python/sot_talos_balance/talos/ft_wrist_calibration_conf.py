import numpy as np

''' *********************** USER-PARAMETERS FOR FT CALIBRATION *********************** '''

handWeight = dict() # Force (N) applied by the hand when horizontal
handWeight['robot'] = [0., 26.93, 0.]
handWeight['simu'] = [0., -14.60, 0.]