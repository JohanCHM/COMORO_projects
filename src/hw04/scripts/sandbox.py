from PID import PID
import numpy as np

kp = np.array([0.05, 0.05, 0.2, 0, 0, 0.2])
# Integral parameters XYZrpy
ki = np.array([0.05, 0.05, 0.2, 0, 0, 0.2])
# Proportional parameters XYZrpy
kd = np.array([0.05, 0.05, 0.2, 0, 0, 0.2])

samplingTime = 0.1  # 100 [ms]

# ----- Initialization stage
controller = PID(kp, ki, kd, samplingTime)  # controller
