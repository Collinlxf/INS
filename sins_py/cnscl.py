import numpy as np
from utils import cros
from glv import glv 

def cnscl(imu, coneoptimal):
    wm = imu[:, :3]
    if coneoptimal == 2:
        dphim = 1 / 12 * cros(glv.wm_1, wm[0])  # Assuming wm_1 is a global variable
        phim = wm[0] + dphim
        dvbm = np.zeros(3)
        if imu.shape[1] >= 6:
            vm = imu[:, 3:6]
            scullm = 1 / 12 * (cros(glv.wm_1, vm[0]) + cros(glv.vm_1, wm[0]))
            rotm = 1.0 / 2 * cros(wm[0], vm[0])
            dvbm = vm[0] + rotm + scullm
            glv.vm_1 = vm[0]
        glv.wm_1 = wm[0]
    else:
        phim = wm[0]
        dvbm = np.zeros(3)
    return phim, dvbm