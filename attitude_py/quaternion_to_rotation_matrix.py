import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def quaternion_to_rotation_matrix(q):
    """
    Convert a quaternion into a rotation matrix.
    
    Parameters:
    q : numpy array
        Quaternion (w, x, y, z)
    
    Returns:
    R : numpy array
        3x3 rotation matrix
    """
    w, x, y, z = q
    R = np.array([
        [1 - 2*y**2 - 2*z**2, 2*x*y - 2*z*w, 2*x*z + 2*y*w],
        [2*x*y + 2*z*w, 1 - 2*x**2 - 2*z**2, 2*y*z - 2*x*w],
        [2*x*z - 2*y*w, 2*y*z + 2*x*w, 1 - 2*x**2 - 2*y**2]
    ])
    return R

def rotate_vector(v, q):
    """
    Rotate a vector by a quaternion.
    
    Parameters:
    v : numpy array
        3D vector
    q : numpy array
        Quaternion (w, x, y, z)
    
    Returns:
    v_rot : numpy array
        Rotated 3D vector
    """
    R = quaternion_to_rotation_matrix(q)
    return np.dot(R, v)

# 初始矢量
v = np.array([1, 0, 0])

# 定义一个四元数（例如，绕z轴旋转90度）
q = np.array([np.cos(np.pi/4), 0, 0, np.sin(np.pi/4)])

# 旋转矢量
v_rot = rotate_vector(v, q)

# 可视化
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# 绘制初始矢量
ax.quiver(0, 0, 0, v[0], v[1], v[2], color='r', label='Original Vector')

# 绘制旋转后的矢量
ax.quiver(0, 0, 0, v_rot[0], v_rot[1], v_rot[2], color='b', label='Rotated Vector')

# 设置轴标签
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')

# 设置图例
ax.legend()

plt.show()