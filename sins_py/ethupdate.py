import numpy as np

def ethupdate(eth, pos, vn):
    eth['pos'] = pos
    eth['vn'] = vn
    eth['sl'] = np.sin(pos[0])
    eth['cl'] = np.cos(pos[0])
    eth['tl'] = eth['sl'] / eth['cl']
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
    eth['g'] = 9.7803267714  # Normal gravity at sea level, needs a proper function for actual gravity calculation
    eth['gn'] = np.array([0, 0, eth['g']])
    eth['gcc'] = eth['gn'] - np.cross(eth['wnien'], vn)
    return eth