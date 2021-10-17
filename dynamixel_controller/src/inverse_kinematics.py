import numpy as np


def joint_angles(pos: list, rot: list, dim: dict) -> list:
    x = pos[0]
    y = pos[1]
    z = pos[2]
    angles = np.zeros(4)

    # Joint angle 1
    angles[0] = np.arctan2(y, x)
    
    # Joint angle 2 & 3
    z_ = z - dim["link1"]
    L1 = dim["link2"]
    L2 = dim["link3"]
    xy = np.sqrt(x**2 + y**2)
    r = np.sqrt(xy**2 + z_**2)
    if r > L1 + L2:
        return None
    phi = np.arctan2(xy, z_)
    psi = np.arccos((r**2 + L1**2 - L2**2) / (2*r*L1))
    print(r**2 + L1**2 - L2**2)
    print(2*r*L1)
    theta1 = phi - psi
    theta2 = np.arccos((z - L1 * np.cos(theta1)) / L2) - theta1 - np.pi
    angles[1] = theta1
    angles[2] = theta2

    print("xy", xy)
    print("r", r)
    print("phi", phi)
    print("psi", psi)
    print("theta1", theta1)
    print("theta2", theta2)
    print("angles", angles)

    # Joint angle 4
    angles[3] = rot[2]

    return -((np.pi - angles) % (2.0 * np.pi) - np.pi)

