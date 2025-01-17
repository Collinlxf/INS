import numpy as np

def read_imu_data(file_path):
    with open(file_path, 'rb') as f:
        imu_data = np.fromfile(f, dtype=np.float64)
    imu_data = imu_data.reshape((-1, 7))
    return imu_data

def read_ref_data(file_path):
    with open(file_path, 'rb') as f:
        ref_data = np.fromfile(f, dtype=np.float64)
    ref_data = ref_data.reshape((-1, 10))
    return ref_data