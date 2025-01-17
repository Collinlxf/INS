import numpy as np
from glv import glv
from attitude import a2qua, q2mat, m2att 

class INS:
    def __init__(self, avp0, ts):
        self.ts = ts
        self.nts = 2 * ts
        self.qnb, self.vn, self.pos = self.setvals(a2qua(avp0[:3]), avp0[3:6], avp0[6:9])
        self.qnb, self.att, self.Cnb = self.attsyn(self.qnb)
        self.Cnb0 = self.Cnb
        self.avp = np.concatenate((self.att, self.vn, self.pos))
        self.eth = self.eth_init(self.pos, self.vn)
        self.Mpv = np.array([
            [1 / self.eth['RMh'], 0, 0],
            [0, 1 / self.eth['clRNh'], 0],
            [0, 0, -1]
        ])
        self.an = np.zeros(3)

    def setvals(self, *args):
        return args

    def attsyn(self, qnb):
        Cnb = q2mat(qnb)
        att = m2att(Cnb)
        return qnb, att, Cnb

    def eth_init(self, pos, vn):
        eth = {
            'Re': glv.Re,
            'e2': glv.e2,
            'wie': glv.wie,
            'pos': pos,
            'vn': vn,
            'sl': np.sin(pos[0]),
            'cl': np.cos(pos[0]),
            'tl': np.sin(pos[0]) / np.cos(pos[0])
        }
        eth['sl2'] = eth['sl'] ** 2
        sq = 1 - eth['e2'] * eth['sl2']
        RN = eth['Re'] / np.sqrt(sq)
        eth['RMh'] = RN * (1 - eth['e2']) / sq + pos[2]
        eth['clRNh'] = eth['cl'] * (RN + pos[2])
        eth['RNh'] = RN + pos[2]
        eth['wnie'] = np.array([eth['wie'] * eth['cl'], 0, -eth['wie'] * eth['sl']])
        eth['wnen'] = np.array([vn[1] / eth['RNh'], -vn[0] / eth['RMh'], -vn[1] / eth['RNh'] * eth['tl']])
        eth['wnin'] = eth['wnie'] + eth['wnen']
        eth['wnien'] = eth['wnie'] + eth['wnin']
        eth['g'] = self.calg(eth['pos'][2], eth['pos'][0])
        eth['gn'] = np.array([0, 0, eth['g']])
        eth['gcc'] = eth['gn'] - np.cross(eth['wnien'], vn)
        return eth

    def calg(self, h, lat):
        # Placeholder for gravitational acceleration calculation
        return glv.g0  # This should be updated with the actual calculation