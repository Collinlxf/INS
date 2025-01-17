import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def euler_to_dcm(att):
    """
    Convert Euler angles to direction cosine matrix (DCM).
    
    Parameters:
    att : list or numpy array
        Euler angles [pitch, roll, yaw] in radians
    
    Returns:
    Cnb : numpy array
        DCM from navigation-frame (n) to body-frame (b), in yaw->pitch->roll (3-1-2) rotation sequence
    """
    # Extract angles
    pitch, roll, yaw = att
    
    # Compute sines and cosines of angles
    s1, s2, s3 = np.sin(att)
    c1, c2, c3 = np.cos(att)

    # Compute the direction cosine matrix
    Cnb = np.array([
        [ c2 * c3, -c1 * s3 + s1 * s2 * c3,  s1 * s3 + c1 * s2 * c3],
        [ c2 * s3,  c1 * c3 + s1 * s2 * s3, -s1 * c3 + c1 * s2 * s3],
        [-s2,       s1 * c2,                c1 * c2]
    ])
    
    return Cnb

def generate_euler_angles(num_samples):
    np.random.seed(0)  # For reproducibility
    pitch = np.random.uniform(-np.pi/6, np.pi/6, num_samples)
    roll = np.random.uniform(-np.pi/6, np.pi/6, num_samples)
    yaw = np.random.uniform(-np.pi/6, np.pi/6, num_samples)
    return np.vstack((pitch, roll, yaw)).T

def plot_attitude(dcm_matrices):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # 绘制初始的 X、Y、Z 轴
    origin = np.array([0, 0, 0])
    ax.quiver(*origin, 1, 0, 0, color='magenta', linewidth=2, label='Initial X-axis')
    ax.quiver(*origin, 0, 1, 0, color='cyan', linewidth=2, label='Initial Y-axis')
    ax.quiver(*origin, 0, 0, 1, color='yellow', linewidth=2, label='Initial Z-axis')

    # 绘制每个姿态的坐标系
    for Cnb in dcm_matrices:
        x_axis = Cnb[:, 0]
        y_axis = Cnb[:, 1]
        z_axis = Cnb[:, 2]
        
        # 绘制X轴方向
        ax.quiver(*origin, *x_axis, color='r')
        # 绘制Y轴方向
        ax.quiver(*origin, *y_axis, color='g')
        # 绘制Z轴方向
        ax.quiver(*origin, *z_axis, color='b')

    # 设置轴标签
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')

    # 设置图例
    ax.legend()

    plt.show()

# 生成模拟的姿态数据并转换
num_samples = 1
euler_angles = generate_euler_angles(num_samples)
dcm_matrices = np.array([euler_to_dcm(att) for att in euler_angles])

# 绘制姿态数据
plot_attitude(dcm_matrices)