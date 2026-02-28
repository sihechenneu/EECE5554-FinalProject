import numpy as np
from scipy.spatial.transform import Rotation as R

def _get_seq(axes):
    if axes.startswith('s'):
        return axes[1:].lower()
    elif axes.startswith('r'):
        return axes[1:].upper()
    return 'xyz'

def euler_matrix(ai, aj, ak, axes='sxyz'):
    r = R.from_euler(_get_seq(axes), [ai, aj, ak])
    mat = np.identity(4)
    mat[:3, :3] = r.as_matrix()
    return mat

def euler_from_matrix(matrix, axes='sxyz'):
    r = R.from_matrix(np.array(matrix)[:3, :3])
    return r.as_euler(_get_seq(axes))

def quaternion_from_euler(ai, aj, ak, axes='sxyz'):
    r = R.from_euler(_get_seq(axes), [ai, aj, ak])
    return r.as_quat()

def euler_from_quaternion(quaternion, axes='sxyz'):
    r = R.from_quat(quaternion)
    return r.as_euler(_get_seq(axes))

def quaternion_matrix(quaternion):
    r = R.from_quat(quaternion)
    mat = np.identity(4)
    mat[:3, :3] = r.as_matrix()
    return mat

def quaternion_from_matrix(matrix):
    r = R.from_matrix(np.array(matrix)[:3, :3])
    return r.as_quat()
