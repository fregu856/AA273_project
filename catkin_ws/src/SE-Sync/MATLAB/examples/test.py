import matlab.engine # (has to be imported first for some reason)
import numpy as np

eng = matlab.engine.start_matlab()

# get the computed estimates:
xhat = eng.test()

raw_translations = xhat["t"]
raw_rotations = xhat["R"]

translations = np.zeros([2, len(raw_translations[0])])

translations[0] = raw_translations[0]
translations[1] = raw_translations[1]

# (the ith column of translations (translations[:, i-1]) is the ith
# translation vector)

rotations = np.zeros([2, len(raw_rotations[0])])

rotations[0] = raw_rotations[0]
rotations[1] = raw_rotations[1]

# (the first two columns of rotations is the first rotation matrix etc.)
