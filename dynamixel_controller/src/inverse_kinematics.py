import numpy as np
np.set_printoptions(precision=2)


LIMIT = 2 * np.pi / 3


def joint_angles(pos: list, rot: list, dim: dict) -> np.array or str:
    """
    Finds the joint angles for the robot arm to position the end-effector at
    the desired location. Uses an analytical inverse kinematics by sectioning
    joint 1 in the x-y plane, joint 2 and 3 in the plane of the z-axis, and
    joint 3 as a wrist rotation.

    Parameters:
        pos (list): The desired position in x,y,z
        rot (list): The desired rotation in x,y,z
        dim (dict): The dimensions of the robot arm according to the
                    Configuration class layout
    
    Returns:
        angles (list): The required joint angles to position the end-effector
                        at the desired location
        msg (str): A message indicating the desired location is not possible
    """
    x = pos[0]
    y = pos[1]
    z = pos[2]
    angles = np.zeros(4)

    # Joint angle 1
    xy_angle = np.arctan2(y, x)
    z_flip = True
    if xy_angle > LIMIT:
        xy_angle -= np.pi
    elif xy_angle < -LIMIT:
        xy_angle += np.pi
    else:
        z_flip = False
    angles[0] = xy_angle
    
    # Joint angle 2 & 3
    z_offset = z - dim["link1"] + dim["link4"]
    L1 = dim["link2"]
    L2 = dim["link3"]
    xy = np.sqrt(x**2 + y**2)
    r = np.sqrt(xy**2 + z_offset**2)
    if r > L1 + L2:
        return f"Reach ({r}) out of range of total link length ({L1+L2})"
    phi = np.arctan2(xy, z_offset)
    alpha = np.arccos((r**2 + L1**2 - L2**2) / (2*r*L1))
    theta1 = phi - alpha
    theta2 = np.arccos((z_offset - L1 * np.cos(theta1)) / L2) - theta1
    angles[1] = theta1
    angles[2] = theta2
    if z_flip:
        angles[1:3] *= -1

    # Joint angle 4
    angles[3] = rot[2] + np.pi / 4

    if not -LIMIT < angles[1] < LIMIT:
        return f"Joint angle 2 ({abs(angles[1])}) exceeding limit ({LIMIT})"
    if not -LIMIT < angles[2] < LIMIT:
        return f"Joint angle 3 ({abs(angles[2])}) exceeding limit ({LIMIT})"
    
    if angles[0] > np.pi:
        angles[0] = 2*np.pi - angles[0]
    return angles
