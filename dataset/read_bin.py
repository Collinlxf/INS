import numpy as np
import pandas as pd
import math

# 定义GCJ-02转换函数
def transform_lat(x, y):
    ret = -100.0 + 2.0 * x + 3.0 * y + 0.2 * y * y + 0.1 * x * y + 0.2 * math.sqrt(abs(x))
    ret += (20.0 * math.sin(6.0 * x * math.pi) + 20.0 * math.sin(2.0 * x * math.pi)) * 2.0 / 3.0
    ret += (20.0 * math.sin(y * math.pi) + 40.0 * math.sin(y / 3.0 * math.pi)) * 2.0 / 3.0
    ret += (160.0 * math.sin(y / 12.0 * math.pi) + 320 * math.sin(y * math.pi / 30.0)) * 2.0 / 3.0
    return ret

def transform_lon(x, y):
    ret = 300.0 + x + 2.0 * y + 0.1 * x * x + 0.1 * x * y + 0.1 * math.sqrt(abs(x))
    ret += (20.0 * math.sin(6.0 * x * math.pi) + 20.0 * math.sin(2.0 * x * math.pi)) * 2.0 / 3.0
    ret += (20.0 * math.sin(x * math.pi) + 40.0 * math.sin(x / 3.0 * math.pi)) * 2.0 / 3.0
    ret += (150.0 * math.sin(x / 12.0 * math.pi) + 300.0 * math.sin(x / 30.0 * math.pi)) * 2.0 / 3.0
    return ret

def wgs84_to_gcj02(lon, lat):
    if out_of_china(lon, lat):
        return lon, lat
    dlat = transform_lat(lon - 105.0, lat - 35.0)
    dlon = transform_lon(lon - 105.0, lat - 35.0)
    radlat = lat / 180.0 * math.pi
    magic = math.sin(radlat)
    magic = 1 - ee * magic * magic
    sqrtmagic = math.sqrt(magic)
    dlat = (dlat * 180.0) / ((a * (1 - ee)) / (magic * sqrtmagic) * math.pi)
    dlon = (dlon * 180.0) / (a / sqrtmagic * math.cos(radlat) * math.pi)
    mglat = lat + dlat
    mglon = lon + dlon
    return mglon, mglat

def out_of_china(lon, lat):
    return not (73.66 < lon < 135.05 and 3.86 < lat < 53.55)

a = 6378245.0
ee = 0.00669342162296594323

def read_imu_data(filename):
    with open(filename, 'rb') as file:
        imu_data = np.fromfile(file, dtype=np.float64)
        trans_imu_data = imu_data.reshape((-1, 7))  # 按照七列进行重塑
    return trans_imu_data

def read_reference_data(filename):
    with open(filename, 'rb') as file:
        ref_imu_data = np.fromfile(file, dtype=np.float64)
        ref_ins_result = ref_imu_data.reshape((-1, 10))  # 按照十列进行重塑
    return ref_ins_result

def save_to_csv(data, filename, headers):
    df = pd.DataFrame(data, columns=headers)
    df.to_csv(filename, index=False)

if __name__ == "__main__":
    imu_filename = 'IMU.bin'
    reference_filename = 'Reference.bin'
    
    # 读取IMU数据
    trans_imu_data = read_imu_data(imu_filename)
    
    # 定义IMU数据的表头
    imu_headers = [
        'timestamp', 
        'gyro_x', 'gyro_y', 'gyro_z', 
        'accel_x', 'accel_y', 'accel_z'
    ]
    
    # 保存IMU数据到CSV文件
    save_to_csv(trans_imu_data, 'IMU_data.csv', imu_headers)
    
    # 读取参考数据
    ref_ins_result = read_reference_data(reference_filename)
    
    # 定义参考数据的表头
    ref_headers = [
        'timestamp', 
        'latitude', 'longitude', 'altitude', 
        'velocity_n', 'velocity_e', 'velocity_d', 
        'roll', 'pitch', 'yaw'
    ]
    
    # 创建GCJ-02列
    gcj02_coords = [wgs84_to_gcj02(lon, lat) for lon, lat in zip(ref_ins_result[:, 2], ref_ins_result[:, 1])]
    gcj02_lon, gcj02_lat = zip(*gcj02_coords)
    ref_ins_result = np.column_stack((ref_ins_result, gcj02_lat, gcj02_lon))
    
    # 更新表头以包含新的GCJ-02列
    ref_headers += ['gcj02_latitude', 'gcj02_longitude']
    
    # 保存参考数据到CSV文件
    save_to_csv(ref_ins_result, 'Reference_data.csv', ref_headers)
    
    print("数据已成功保存到CSV文件！")