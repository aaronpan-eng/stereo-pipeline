import pickle
import numpy as np

# Load the calibration file
with open('/workspace/data/2025-10-PRANCE/calibration_2025-10-15_171445/output/stereo_calibration_2024-10-28.pckl', 'rb') as f:
    calib_data = pickle.load(f)

# Check what's inside
print(type(calib_data))
print(calib_data.keys() if isinstance(calib_data, dict) else calib_data)
print(calib_data)