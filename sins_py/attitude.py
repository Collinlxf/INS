import numpy as np

def a2mat(att):
    s = np.sin(att)
    c = np.cos(att)
    s1, s2, s3 = s[0], s[1], s[2]
    c1, c2, c3 = c[0], c[1], c[2]
    Cnb = np.array([
        [c2*c3, -c1*s3 + s1*s2*c3, s1*s3 + c1*s2*c3],
        [c2*s3, c1*c3 + s1*s2*s3, -s1*c3 + c1*s2*s3],
        [-s2, s1*c2, c1*c2]
    ])
    return Cnb

def a2qua(att):
    att2 = att / 2
    s = np.sin(att2)
    c = np.cos(att2)
    s1, s2, s3 = s[0], s[1], s[2]
    c1, c2, c3 = c[0], c[1], c[2]
    qnb = np.array([
        c1*c2*c3 + s1*s2*s3,
        s1*c2*c3 - c1*s2*s3,
        c1*s2*c3 + s1*c2*s3,
        c1*c2*s3 - s1*s2*c3
    ])
    return qnb

def m2att(Cnb):
    att = np.array([
        np.arctan2(Cnb[2, 1], Cnb[2, 2]),
        np.arctan(-Cnb[2, 0] / (np.sqrt(Cnb[2, 1]**2 + Cnb[2, 2]**2))),
        np.arctan2(Cnb[1, 0], Cnb[0, 0])
    ])
    return att

def m2qua(Cnb):
    att = m2att(Cnb)
    qnb = a2qua(att)
    return qnb

def q2mat(qnb):
    q11 = qnb[0] * qnb[0]
    q12 = qnb[0] * qnb[1]
    q13 = qnb[0] * qnb[2]
    q14 = qnb[0] * qnb[3]
    q22 = qnb[1] * qnb[1]
    q23 = qnb[1] * qnb[2]
    q24 = qnb[1] * qnb[3]
    q33 = qnb[2] * qnb[2]
    q34 = qnb[2] * qnb[3]
    q44 = qnb[3] * qnb[3]
    Cnb = np.array([
        [q11 + q22 - q33 - q44, 2*(q23 - q14), 2*(q24 + q13)],
        [2*(q23 + q14), q11 - q22 + q33 - q44, 2*(q34 - q12)],
        [2*(q24 - q13), 2*(q34 + q12), q11 - q22 - q33 + q44]
    ])
    return Cnb

def qmulv(q, vi):
    qo1 = -q[1] * vi[0] - q[2] * vi[1] - q[3] * vi[2]
    qo2 = q[0] * vi[0] + q[2] * vi[2] - q[3] * vi[1]
    qo3 = q[0] * vi[1] + q[3] * vi[0] - q[1] * vi[2]
    qo4 = q[0] * vi[2] + q[1] * vi[1] - q[2] * vi[0]
    vo = np.zeros(3)
    vo[0] = -qo1 * q[1] + qo2 * q[0] - qo3 * q[3] + qo4 * q[2]
    vo[1] = -qo1 * q[2] + qo3 * q[0] - qo4 * q[1] + qo2 * q[3]
    vo[2] = -qo1 * q[3] + qo4 * q[0] - qo2 * q[2] + qo3 * q[1]
    return vo

def qupdt2(qnb0, rv_ib, rv_in):
    # Quaternion updating using rotation vector
    n2 = rv_ib[0]**2 + rv_ib[1]**2 + rv_ib[2]**2
    if n2 < 1.0e-8:
        rv_ib0 = 1 - n2 * (1/8 - n2/384)
        s = 1/2 - n2 * (1/48 - n2/3840)
    else:
        n = np.sqrt(n2)
        n_2 = n / 2
        rv_ib0 = np.cos(n_2)
        s = np.sin(n_2) / n
    
    rv_ib = s * rv_ib
    qb1 = qnb0[0] * rv_ib0 - qnb0[1] * rv_ib[0] - qnb0[2] * rv_ib[1] - qnb0[3] * rv_ib[2]
    qb2 = qnb0[0] * rv_ib[0] + qnb0[1] * rv_ib0 + qnb0[2] * rv_ib[2] - qnb0[3] * rv_ib[1]
    qb3 = qnb0[0] * rv_ib[1] + qnb0[2] * rv_ib0 + qnb0[3] * rv_ib[0] - qnb0[1] * rv_ib[2]
    qb4 = qnb0[0] * rv_ib[2] + qnb0[3] * rv_ib0 + qnb0[1] * rv_ib[1] - qnb0[2] * rv_ib[0]
    
    n2 = rv_in[0]**2 + rv_in[1]**2 + rv_in[2]**2
    if n2 < 1.0e-8:
        rv_in0 = 1 - n2 * (1/8 - n2/384)
        s = -1/2 + n2 * (1/48 - n2/3840)
    else:
        n = np.sqrt(n2)
        n_2 = n / 2
        rv_in0 = np.cos(n_2)
        s = -np.sin(n_2) / n
    
    rv_in = s * rv_in
    qnb1 = np.array([
        rv_in0 * qb1 - rv_in[0] * qb2 - rv_in[1] * qb3 - rv_in[2] * qb4,
        rv_in0 * qb2 + rv_in[0] * qb1 + rv_in[1] * qb4 - rv_in[2] * qb3,
        rv_in0 * qb3 + rv_in[1] * qb1 + rv_in[2] * qb2 - rv_in[0] * qb4,
        rv_in0 * qb4 + rv_in[2] * qb1 + rv_in[0] * qb3 - rv_in[1] * qb2
    ])
    
    norm = np.linalg.norm(qnb1)
    if norm > 1.000001 or norm < 0.999999:
        qnb1 /= norm
    return qnb1

def attsyn(qnb):
    Cnb = q2mat(qnb)
    att = m2att(Cnb)
    return qnb, att, Cnb