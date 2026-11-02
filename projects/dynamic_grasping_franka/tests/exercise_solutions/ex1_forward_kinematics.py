import numpy as np
from kinematics import S_LIST, M_HOME, exp_twist


def forward_kinematics(q):
    T = np.eye(4)
    for i in range(7):
        T = T @ exp_twist(S_LIST[:, i], q[i])
    return T @ M_HOME
