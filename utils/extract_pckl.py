import sys
import pickle
import numpy as np


def extract_from_pckl(file):
    with open(file, 'rb') as f:
        calib_data = pickle.load(f)

    # define pckl format
    format = [
        "rms_error",
        "K1",
        "D1",
        "K2",
        "D2",
        "R",
        "T",
        "E",
        "F"
    ]

    calib_dict = {}
    # Create dictionary for calibration
    for i, value in enumerate(calib_data):
        calib_dict[format[i]] = value
    
    return calib_dict

if __name__ == "__main__":
    # filename and grab dict
    file = '/workspace/data/2025-10-PRANCE/calibration_2025-10-15_171445/output/stereo_calibration_2024-10-28.pckl'
    calib_dict = extract_from_pckl(file)

    # print out file format
    for key, value in calib_dict.items():
        print(f"{key}:\n{value}")
