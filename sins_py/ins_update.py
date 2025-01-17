import numpy as np
from utils import cros, setMat, rv2q
from attitude import qmulv, qupdt2, attsyn
from cnscl import cnscl  
from ethupdate import ethupdate

def ins_update_alog(ins, imu):
    nn = imu.shape[0]
    nts = nn * ins.ts
    nts2 = nts / 2
    ins.nts = nts
    phim, dvbm = cnscl(imu, 2)  # Coning & sculling compensation

    # Earth & angular rate updating
    vn01 = ins.vn + ins.an * nts2
    pos01 = ins.pos + ins.Mpv @ vn01 * nts2
    ins.eth = ethupdate(ins.eth, pos01, vn01)
    ins.wib = phim / nts
    ins.fb = dvbm / nts

    # Velocity updating
    ins.fn = qmulv(ins.qnb, ins.fb)
    ins.an = qmulv(rv2q(-ins.eth['wnin'] * nts2), ins.fn) + ins.eth['gcc']
    vn1 = ins.vn + ins.an * nts

    # Position updating
    ins.Mpv = np.array([
        [1 / ins.eth['RMh'], 0, 0],
        [0, 1 / ins.eth['clRNh'], 0],
        [0, 0, -1]
    ])
    ins.Mpvvn = ins.Mpv @ (ins.vn + vn1) / 2
    ins.pos1 = ins.pos + ins.Mpvvn * nts

    # Attitude updating
    ins.Cnb0 = ins.Cnb
    ins.qnb = qupdt2(ins.qnb, phim, ins.eth['wnin'] * nts)
    ins.qnb, ins.att, ins.Cnb = attsyn(ins.qnb)

    # 整理最终结果
    ins.avp = np.concatenate((ins.att, ins.vn, ins.pos))
    return ins