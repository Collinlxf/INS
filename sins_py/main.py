import numpy as np
from glv import glv
from ins import INS
from data_read import read_imu_data, read_ref_data
from attitude import a2mat, a2qua, m2att, m2qua, q2mat
from ins_update import ins_update_alog  # Ensure this import is correct
import matplotlib.pyplot as plt

# 声明全局变量并添加搜索路径
glv = glv

# 测试四元数和欧拉角、DCM之间的转换是否正确
att = np.array([-0.5, 0.3, -1])
att1 = m2att(q2mat(a2qua(att)))
qu1 = m2qua(a2mat(att))
qu2 = a2qua(att)

# 读取标定数据
isRead = 0
if isRead != 1:
    # 读取惯导数据
    imu_data = read_imu_data('IMU.bin')
    trans_imu_data = imu_data[:, 1:7]
    trans_imu_data = np.hstack((trans_imu_data, imu_data[:, 0].reshape(-1, 1)))

    # 读取参考结果
    ref_ins_result = read_ref_data('Reference.bin')

# 北东地坐标系
first_pos = np.where(trans_imu_data[:, 6] == 91620)[0][0]
imu_data = trans_imu_data[first_pos + 1:, :]

ts = 0.005  # 采样间隔
avp0 = np.array([
    0.0107951084511778 * glv.deg, -2.14251290749072 * glv.deg, -75.7498049314083 * glv.deg,
    0, 0, 0,
    23.1373950708 * glv.deg, 113.3713651222 * glv.deg, 2.175
])
ins = INS(avp0, ts)

nn = 1
len_imu = len(imu_data)
avp = np.zeros((len_imu // nn, 10))

# 核心部分，开始进行计算了
for k in range(0, len_imu - nn + 1, nn):
    k1 = k + nn - 1
    wvm = imu_data[k:k1 + 1, :6]
    t = imu_data[k1, 6]
    ins = ins_update_alog(ins, wvm)
    avp[k // nn, :] = np.hstack((ins.avp, t))

# 和参考计算结果做对比
trans_avp = np.hstack((
    avp[:, 9].reshape(-1, 1), avp[:, 6:8] / glv.deg, avp[:, 8].reshape(-1, 1),
    avp[:, 3:6], avp[:, 0:3] / glv.deg
))
t = trans_avp[:, 0]
err_with_ref = trans_avp - ref_ins_result

# 绘图
import matplotlib.pyplot as plt

plt.figure()
plt.subplot(311)
plt.plot(t, err_with_ref[:, 7:10], '-', linewidth=2)
plt.ylabel('phi_degree')
plt.legend(['phi_roll', 'phi_pitch', 'phi_yaw'])
plt.grid()

plt.subplot(312)
plt.plot(t, err_with_ref[:, 4:7], '-', linewidth=2)
plt.ylabel('dV')
plt.legend(['dVN', 'dVE', 'dVD'])
plt.grid()

plt.subplot(313)
plt.plot(t, err_with_ref[:, 1:3], '-', linewidth=2)
plt.ylabel('dP_latlon')
plt.legend(['dlat', 'dlon'])
plt.grid()

plt.figure()
errN = err_with_ref[:, 1] * glv.deg * ins.eth['RMh']
errE = err_with_ref[:, 2] * glv.deg * ins.eth['clRNh']
errD = -err_with_ref[:, 3]
errNED = np.vstack((errN, errE, errD)).T
plt.plot(t, errNED[:, 0:3], '-', linewidth=2)
plt.ylabel('dP')
plt.legend(['dN', 'dE', 'dD'])
plt.grid()
plt.show()