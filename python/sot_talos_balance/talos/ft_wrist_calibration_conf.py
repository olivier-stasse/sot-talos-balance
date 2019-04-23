import numpy as np

''' *********************** USER-PARAMETERS FOR FT CALIBRATION *********************** '''

handWeight = dict() # Force (N) applied by the hand when horizontal
handWeight['robot'] = [0., 0., -26.93]
handWeight['simu'] = [0., 0., -14.6048]
LEFT_LEVER_ARM = [0.00198151, -0.0041245, -0.07866962]
RIGHT_LEVER_ARM = [-0.0165427, -0.00574413, -0.0792288]
