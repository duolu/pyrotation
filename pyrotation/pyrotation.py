'''
This file is part of the pyrotation python module, which is designed to help
teaching and learning the math of 3D rotation. This file contains the core
class and functions of 3D rotation.

Author: Duo Lu <duolu.cs@gmail.com>

Version: 0.1
License: GPLv3

Updated on Jan. 28, 2020, version 0.1

Created on Mar 22, 2019, draft


'''

import math
from math import copysign, fabs, sqrt, pi, sin, cos, asin, acos, atan2, exp, log
import numpy as np

# --------------------- common utilities ----------------------------

# This is the floating point error tolerance.
EPSILON = 1e-6



def vector_to_skew_symmetric_matrix(v):
    '''
    Return the skew matrix M constructed by vector v, i.e.,
    
    Assume:
    
        v = (x, y, z)
    
    Then:
    
        M = [  0, -z,  y ]
            [  z,  0, -x ]
            [ -y,  x,  0 ]
    
    '''
    
    x = v[0]
    y = v[1]
    z = v[2]
    
    M = np.asarray((
        (  0, -z,  y ),
        (  z,  0, -x ),
        ( -y,  x,  0 )
        ))

    return M

def skew_symmetric_matrix_to_vector(M):
    '''
    Return the vector that corresponds to the skew matrix M.
    
    Assume:
    
        m = [  0, -z,  y ]
            [  z,  0, -x ]
            [ -y,  x,  0 ]

    Then:
    
        v = (x, y, z)
    
    
    CAUTION: this function does not check whether the matrix is really a skew
    symmetric matrix! It assumes the input is correct! The input must be a
    numpy 3-by-3 matrix!
    
    '''

    x = M[2, 1]
    y = M[0, 2]
    z = M[1, 0]
    
    return np.asarray((x, y, z))


# --------------------- angle-axis / alt-azimuth-angle --------------------


def alt_azimuth_to_axis(alt_degree, azimuth_degree):
    '''
    Calculate an axis represented by a unit vector using alt and azimuth 
    angles.
    
    NOTE: alt and azimuth are in degree, not radian. The output axis
    is always a unit vector.
    '''
    
    alt_radian = alt_degree * np.pi / 180
    azimuth_radian = azimuth_degree * np.pi / 180
    
    ux = cos(alt_radian) * cos(azimuth_radian)
    uy = cos(alt_radian) * sin(azimuth_radian)
    uz = sin(alt_radian)
    
    u = np.asarray((ux, uy, uz))
    u = u / np.linalg.norm(u)

    return u

def axis_to_alt_azimuth(u):
    '''
    Calculate the alt and azimuth angles from an axis vector "u".
    
    NOTE: alt and azimuth are in degree, not radian.
    
    CAUTION: In the degenerated case, where the norm of the vector u is zero,
    i.e., the vector u is pointting to the z-axis or the opposite of the 
    z-axis, the output alt is set to plus/minus 90 degrees and the azimuth to
    zero degrees.
    
    '''
    
    u_norm = np.linalg.norm(u)
    
    if fabs(fabs(u_norm) - 1) < EPSILON:
        
        degenerated = True

        alt = 90
        azimuth = 0
        
        gimbal_lock = False
        
    else:
        
        degenerated = False
        
        u = u / u_norm
    
        if fabs(u[2] - 1) < EPSILON:
            
            # CAUTION: if u is pointing to the z-axis, the result is degenerated,
            # i.e., azimuth is undefined.
            
            alt = 90
            azimuth = 0
            
            gimbal_lock = True
            
        elif fabs(u[2] + 1) < EPSILON:
            
            # Similarly, if u is pointing to the opposite of the z-axis, azimuth is
            # also undefined.
            
            alt = -90
            azimuth = 0
    
            gimbal_lock = True
            
        else:
            
            alt = asin(u[2])
            
            c = cos(alt)
            
            dx = u[0] / c
            dy = u[1] / c
    
            azimuth = atan2(dy, dx)
            
            alt = alt * 180 / pi
            azimuth = azimuth * 180 / pi
            
            gimbal_lock = False

    return alt, azimuth, gimbal_lock, degenerated

def rotate_a_point_by_angle_axis(p, u):
    '''
    Rotate a point "p" along the axis "u" by an angle which is the norm of "u".
    Both "p" and "u" are numpy arrays of three elements.
    
    CAUTION: For numerical stability, small angle rotations are considered
    as no rotation (i.e., if angle < 1e-6 radian, angle is considered as zero).
    
    '''

    angle = np.linalg.norm(u)

    if fabs(angle) < EPSILON:
        
        return p
    
    else:

        # normalize the axis vector
        u = u / np.linalg.norm(u)
    
        pp = np.dot(p, u) * u
        pv = p - pp
        
        u_x_cross = np.cross(u, p)
        
        rp = pp + pv * np.cos(angle) + u_x_cross * np.sin(angle)
    
        return rp

def rotate_points_by_angle_axis(ps, u):
    '''
    Rotate points "ps" along the axis "u" by an angle which is the norm of "u".
    "ps" must a numpy 3-by-n array, and u must a numpy array of three elements.
    
    CAUTION: For numerical stability, small angle rotations are considered
    as no rotation (i.e., if angle < 1e-6 radian, angle is considered as zero).
    
    CAUTION: The input "ps" is now an array of vectors, where each vector is a
    column.
    
    '''

    angle = np.linalg.norm(u)

    if fabs(angle) < EPSILON:
        
        return ps
    
    else:

        # normalize the axis vector
        u = u / np.linalg.norm(u)
    
        u = u.reshape((3, 1))
    
        # CAUTION: since ps is an array of vectors, pp should be the dot
        # product of each vector with u to obtain an array of scalars, then
        # multiply each scalar with u to form an array of vectors.
        pp = np.matmul(u, np.matmul(u.T, ps))
        pv = ps - pp

        # CAUTION: numpy uses the last axis to define vectors in the cross 
        # product with arrays of vectors
        u_x_cross = np.cross(u.T, ps.T).T
        
        rps = pp + pv * np.cos(angle) + u_x_cross * np.sin(angle)
    
        return rps


def angle_axis_to_rotation_matrix(u):
    '''
    Convert angle-axis representation of rotation to a rotation matrix, using
    the Rodrigues formula.
    
    CAUTION: For numerical stability, small angle rotations are considered
    as no rotation (i.e., if angle < 1e-6 radian, angle is considered as zero).
    '''
    
    angle = np.linalg.norm(u)
    
    I = np.identity(3)
    
    if angle < EPSILON:
        
        return I
    
    else:
    
        # normalize the axis vector
        u = u / np.linalg.norm(u)
        
        
        ux = vector_to_skew_symmetric_matrix(u)
        
        uut = np.matmul(u.reshape((3, 1)), u.reshape(1, 3))
        
        
        R = I * cos(angle) + ux * sin(angle) + uut * (1 - cos(angle))
    
        return R


def rotation_matrix_to_angle_axis(R):
    '''
    Convert a rotation matrix to an angle-axis representation.
    
    NOTE: The returned rotation angle is between 0 and pi.
    
    CAUTION: For numerical stability, small angle rotations are considered
    as no rotation.
    
    CAUTION: In the degenerated case, where the rotation angle is zero, the
    axis is set to (0, 0, 1), i.e., the z-axis.
    
    '''
    
    # TODO: this might be buggy!!!
    
    # Check the determinant of R! It must be 1.
    assert fabs(np.linalg.det(R) - 1) < EPSILON
    assert np.allclose(np.matmul(R, R.T), np.identity(3))

    
    
    if np.allclose(R, np.identity(3)):
        
        u = np.asarray((0, 0, 0))
    
    else:
        
        angle = acos((np.trace(R) - 1) / 2)
        
        s = 2 * sin(angle)
        
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s
    
        u = np.asarray((x, y, z))
        u = u / np.linalg.norm(u)
        u = u * angle
    
    return u






# --------------------- Euler angles ------------------


def euler_zyx_to_rotation_matrix(z, y, x):
    '''
    Converting a rotation represented by three Euler angles (z-y'-x") to
    rotation matrix represenation, i.e., using the following,
    
        R = R_z * R_y * R_x
    
    where,
    
        R_z             = [ cos(z)     -sin(z)        0       ]
                          [ sin(z)     cos(z)         0       ]
                          [ 0            0            1       ]
    
        R_y             = [ cos(y)       0            sin(y)  ]
                          [ 0            1            0       ]
                          [ -sin(y)      0            cos(y)  ]
    
        R_x             = [ 1            0            0       ]
                          [ 0            cos(x)       -sin(x) ]
                          [ 0            sin(x)       cos(x)  ]
    
    Also, the angles are named as following,
    
        z - yaw (psi)
        y - pitch (theta)
        x - roll (phi)
    
    These angles are also called Tait-Bryan angles, and we use the z-y'-x"
    intrinsic convention. See this for the conventions:
    
        https://en.wikipedia.org/wiki/Euler_angles#Tait–Bryan_angles
    
    Also see this for the conversion between different representations:
    
        https://en.wikipedia.org/wiki/Rotation_formalisms_in_three_dimension
    
    Caution: The three input angles are in radian!
    
    '''
    

    sz = np.sin(z)
    cz = np.cos(z)
    sy = np.sin(y)
    cy = np.cos(y)
    sx = np.sin(x)
    cx = np.cos(x)

    a11 = cz * cy
    a12 = cz * sy * sx - cx * sz
    a13 = sz * sx + cz * cx * sy
    a21 = cy * sz
    a22 = cz * cx + sz * sy * sx
    a23 = cx * sz * sy - cz * sx
    a31 = -sy
    a32 = cy * sx
    a33 = cy * cx

    R = np.asarray([[a11, a12, a13], [a21, a22, a23], [a31, a32, a33]])

    return R

def rotation_matrix_to_euler_angles_zyx(R):
    '''
    Converting a rotation represented by three Euler angles (z-y'-x") to
    rotation matrix represenation.

    CAUTION: Euler angles have a singularity when pitch = pi / 2 or - pi / 2, 
    i.e., gimbal lock. In this case, yaw and roll angles can not be determined
    uniquely, and this function always return a zero yaw angle.
    
    '''
    
    if fabs(fabs(R[2, 0]) - 1) < EPSILON:
        
        # cos(y) != 0, gimbal lock
    
        # CAUTION: y is always pi/2, and z is always 0
        y = copysign(pi / 2, -R[2, 0])
    
        x = 0
        z = atan2(R[0, 1], R[0, 2])
        
        gimbal_lock = True
        
        #print('gimbal lock!!!')
        
    else:
        
        # cos(y) == 0, normal situation
        
        # CAUTION: y is always in [-pi/2, pi/2]
        y = -asin(R[2, 0])
        cy = cos(y)
        x = atan2(R[2, 1] / cy, R[2, 2] / cy)
        z = atan2(R[1, 0] / cy, R[0, 0] / cy)
    
        gimbal_lock = False
    
    return z, y, x, gimbal_lock



# --------------------- Rotation Matrix ------------------




def rotate_a_point_by_rotation_matrix(R, p):
    '''
    Rotate a point "p" by a rotation matrix "R". "p" must be a numpy array
    of three elements. "R" must be a numpy 3-by-3 matrix. The result is a
    numpy array of three elements, same as the input "p".
    
    CAUTION: This function does not validate whether R is really a rotation
    matrix. It just do the following operations.
    
        rp = np.matmul(R, p.reshape(3, 1))
        
        return rp.flatten()
    
    '''
    rp = np.matmul(R, p.reshape(3, 1))

    return rp.flatten()

def rotate_points_by_rotation_matrix(R, ps):
    '''
    Rotate points array "ps" by a rotation matrix "R". "ps" must be a numpy 
    3-by-n matrix where. "R" must be a numpy 3-by-3 matrix. The result is
    a numpy 3-by-n matrix, same as the input "ps".

    CAUTION: This function does not validate whether R is really a rotation
    matrix. It just do the following operations.
    
        rps = np.matmul(R, ps)
        
        return rps
    
    '''
    
    rps = np.matmul(R, ps)

    return rps

def normalize_rotation_matrix(R):
    '''
    Normalize a rotation matrix with SVD, i.e., using the following step.
    
        u, s, vh = np.linalg.svd(R)
        
        return np.matmul(u, vh)
    
    '''
    
    u, s, vh = np.linalg.svd(R)
    del s
    
    return np.matmul(u, vh)




def orthonormal_basis_from_two_vectors(v1, v2, v1_default, v2_default):
    '''
    Construct three orthonormal basis vectors from arbitrary two vectors.
    The output orthonormal basis (u1, u2, u3) has the following properties.
    
    (1) The vector u1 is in the direction of v1. If v1 is all zero, v1_default
    is used instead.
    
    (2) The vector u2 is in the same plane spanned by v1 and v2, and u2 is
    perpendicular to v1, i.e., their dot product is zero. If v2 is all zero
    or if v2 has the same direction as v1, v2_default is used instead.
    
    (3) The vector u3 is perpendicular to the plane spanned by v1 and v2.
    
    '''
    n1 = np.linalg.norm(v1)
    n2 = np.linalg.norm(v2)
    v3 = np.cross(v1, v2)
    n3 = np.linalg.norm(v3)

    if fabs(n1) < EPSILON:
        
        v1 = v1_default
    
    if fabs(n2) < EPSILON or fabs(n3) < EPSILON:

        v2 = v2_default
    
    u1 = v1 / np.linalg.norm(v1)
    v3 = np.cross(u1, v2)
    u3 = v3 / np.linalg.norm(v3)
    v2 = np.cross(u3, u1)
    u2 = v2 / np.linalg.norm(v2)
    
    return u1, u2, u3

def rotation_matrix_from_orthonormal_basis(ux, uy, uz):
    '''
    Construct a 3-by-3 rotation matrix from a set of orthonormal basis vectors.
    
    '''
    
    R = np.zeros((3, 3))
    R[:, 0] = ux
    R[:, 1] = uy
    R[:, 2] = uz

    return R

def rotation_matrix_from_xy(x, y):
    '''
    Construct three orthonormal basis, i.e., the rotation matrix, based on 
    two vectors, where one of them is the x-axis and the other is the y-axis.
    The z-axis is chosen accordingly.
    
    The x-axis is always preserved, and if the y-axis is not perpendicular to
    the x-axis, its projection on the YOZ plane defined by the x-axis will be
    the y-axis.
    
    NOTE: If the vector x has zero length, it is a degenerated case and it is 
    set to (1, 0, 0)
    
    NOTE: If the vector y has zero length or if the vector y is in the same
    direction of the vector x, it is a degenerated case and the vector y is
    set to (0, 1, 0).
    
    CAUTION: No warning or exception is generated in the degenerated case!
    
    '''
    
    v1 = x
    v2 = y
    v1_default = np.asarray((1, 0, 0))
    v2_default = np.asarray((0, 1, 0))
    u1, u2, u3 = orthonormal_basis_from_two_vectors(v1, v2, 
                                                    v1_default, v2_default)
    
    R = rotation_matrix_from_orthonormal_basis(u1, u2, u3)
    
    return R

def rotation_matrix_from_yz(y, z):
    '''
    Construct three orthonormal basis, i.e., the rotation matrix, based on 
    two vectors, where one of them is the y-axis and the other is the z-axis.
    The x-axis is chosen accordingly.
    
    The y-axis is always preserved, and if the z-axis is not perpendicular to
    the y-axis, its projection on the ZOX plane defined by the y-axis will be
    the z-axis.
    
    NOTE: If the vector y has zero length, it is a degenerated case and it is 
    set to (0, 1, 0)
    
    NOTE: If the vector z has zero length or if the vector z is in the same
    direction of the vector y, it is a degenerated case and the vector z is
    set to (0, 0, 1).
    
    CAUTION: No warning or exception is generated in the degenerated case!
    
    '''

    v1 = y
    v2 = z
    v1_default = np.asarray((0, 1, 0))
    v2_default = np.asarray((0, 0, 1))
    u1, u2, u3 = orthonormal_basis_from_two_vectors(v1, v2, 
                                                    v1_default, v2_default)
    
    R = rotation_matrix_from_orthonormal_basis(u3, u1, u2)
    
    return R

def rotation_matrix_from_zx(z, x):
    '''
    Construct three orthonormal basis, i.e., the rotation matrix, based on 
    two vectors, where one of them is the y-axis and the other is the z-axis.
    The x-axis is chosen accordingly.
    
    The y-axis is always preserved, and if the z-axis is not perpendicular to
    the y-axis, its projection on the ZOX plane defined by the y-axis will be
    the z-axis.
    
    NOTE: If the vector y has zero length, it is a degenerated case and it is 
    set to (0, 1, 0)
    
    NOTE: If the vector z has zero length or if the vector z is in the same
    direction of the vector y, it is a degenerated case and the vector z is
    set to (0, 0, 1).
    
    CAUTION: No warning or exception is generated in the degenerated case!
    
    '''
    
    v1 = z
    v2 = x
    v1_default = np.asarray((0, 0, 1))
    v2_default = np.asarray((1, 0, 0))
    u1, u2, u3 = orthonormal_basis_from_two_vectors(v1, v2, 
                                                    v1_default, v2_default)
    
    R = rotation_matrix_from_orthonormal_basis(u2, u3, u1)
    
    return R





# --------------------- quaternion ----------------------------


class Quaternion(object):
    '''
    The class representing a quaternion in Hamiltonian convention.
    
    NOTE: The Quaternion object is designed to be immutable after construction.
        
    '''


    def __init__(self, w, x, y, z):
        '''
        Constructor, wchich expects the four components of a quaternion. 
        
        '''
        
        self.w = float(w)
        self.x = float(x)
        self.y = float(y)
        self.z = float(z)
    
    def __str__(self):
        
        return 'Quaternion(%.3f, %.3f, %.3f, %.3f)' \
            % (self.w, self.x, self.y, self.z)
    
    def __repr__(self):
        
        return 'Quaternion(%.3f, %.3f, %.3f, %.3f)' \
            % (self.w, self.x, self.y, self.z)
    
    
    def to_vector(self):
        
        return np.asarray((self.w, self.x, self.y, self.z))
    
    def add_quaternion(self, q):
        '''
        Quaternion addition.
        
        NOTE: Do not use this when dealing with unit quaternion, since unit
        quaterion is not closed under addition.
        
        NOTE: A new quaternion object is returned as the result. This 
        quaternion is not changed.
        
        '''
        
        w_add = self.w + q.w
        x_add = self.x + q.x
        y_add = self.y + q.y
        z_add = self.z + q.z
    
        return Quaternion(w_add, x_add, y_add, z_add)

    def subtract_quaternion(self, q):
        '''
        Quaternion subtraction.
        
        NOTE: Do not use this when dealing with unit quaternion, since unit
        quaterion is not closed under addition.
        
        NOTE: A new quaternion object is returned as the result. This 
        quaternion is not changed.
        
        '''
        
        w_sub = self.w - q.w
        x_sub = self.x - q.x
        y_sub = self.y - q.y
        z_sub = self.z - q.z
    
        return Quaternion(w_sub, x_sub, y_sub, z_sub)
    
    def multiply_scaler(self, s):
        '''
        Multiply each component with a scalar.
        
        NOTE: Do not use this when dealing with unit quaternion, since unit
        quaterion is not closed under scaling.
        
        NOTE: A new quaternion object is returned as the result. This 
        quaternion is not changed.
        
        '''
        
        return Quaternion(self.w * s, self.x * s, self.y * s, self.z * s)
    
    def multiply_quaternion(self, q):
        '''
        Quaternion multiplication.
        
        Notation: r = p x q is r = p.multiply_quaternion(q), 
        where "x" is quaternion multiplication.
        
        NOTE: A new quaternion object is returned as the result. This 
        quaternion is not changed.
        
        '''
        
        if not isinstance(q, Quaternion):
            
            raise 'Can not multiply non-quaternion! q=%s' % repr(q)
        
        
        pw = self.w
        px = self.x
        py = self.y
        pz = self.z
        
        qw = q.w
        qx = q.x
        qy = q.y
        qz = q.z
        
        w_mul = pw * qw - px * qx - py * qy - pz * qz
        x_mul = pw * qx + px * qw + py * qz - pz * qy
        y_mul = pw * qy - px * qz + py * qw + pz * qx
        z_mul = pw * qz + px * qy - py * qx + pz * qw
    
        return Quaternion(w_mul, x_mul, y_mul, z_mul)
    
    def conjugate(self):
        '''
        Return the conjugate of this quaternion.
        
        Notation: q* is q.conjugate()
        
        NOTE: A new quaternion object is returned as the result. This 
        quaternion is not changed.
        
        '''
        
        return Quaternion(self.w, -self.x, -self.y, -self.z)

    def norm(self):
        '''
        Return the norm of this quaternion.
        
        Notation: |q| is q.norm()
        
        '''
        w = self.w
        x = self.x
        y = self.y
        z = self.z
        
        return sqrt(w * w + x * x + y * y + z * z)

    def normalize(self):
        '''
        Return a normalized unit quaternion based on this quaternion.
        
        NOTE: A new quaternion object is returned as the result. This 
        quaternion is not changed.
        
        '''
        n = self.norm()
        
        w = self.w / n
        x = self.x / n
        y = self.y / n
        z = self.z / n
        
        return Quaternion(w, x, y, z)

    def inverse(self):
        '''
        Return the inverse of this quaternion.
        
        Notation: q-1 is q.inverse()

        NOTE: A new quaternion object is returned as the result. This 
        quaternion is not changed.
        
        '''

        w = self.w
        x = self.x
        y = self.y
        z = self.z
        
        a = 1 / (w * w + x * x + y * y + z * z)

        return Quaternion(self.w * a, -self.x * a, -self.y * a, -self.z * a)

    def negate(self):
        
        return Quaternion(-self.w, -self.x, -self.y, -self.z)
    
    def to_angle_axis(self):
        '''
        Convert this quaternion to angle-axis representation of rotation.
        
        This is the reverse of construct_from_angle_axis()

        This is essentially the logarithmic map.
        
        See Joan Solà, "Quaternion Kinematics", section 2.4, eqation (106).

        CAUTION: This must be a unit quaternion!
        
        '''
        
        assert (fabs(self.norm() - 1) < EPSILON)
        
        w = self.w
        x = self.x
        y = self.y
        z = self.z

        return self.quaternion_to_angle_axis(w, x, y, z)
    
    def to_rotation_matrix(self):
        '''
        Convert this unit quaternion to a rotation matrix.
        
        CAUTION: This must be a unit quaternion!
        
        '''
        
        assert (fabs(self.norm() - 1) < EPSILON)
        
        w = self.w
        x = self.x
        y = self.y
        z = self.z

        return self.quaternion_to_rotation_matrix(w, x, y, z)
    
    def to_left_matrix(self):
        '''
        Return the left matrix of this quaternion.
        
        Notation: p_L = p.to_left_matrix()
        
        p x q = p_L * q, where "*" is matrix multiplication, and q can be
        considered as a 4 by 1 matrix.
        
        '''
        w = self.w
        x = self.x
        y = self.y
        z = self.z

        M = np.asarray((
            ( w, -x, -y, -z ),
            ( x,  w, -z,  y ),
            ( y,  z,  w, -x ),
            ( z, -y,  x,  w )
            ))
        
        return M
    
    def to_right_matrix(self):
        '''
        Return the right matrix of this quaternion.
        
        Notation: q_R is q.to_right_matrix()
        
        p x q = q_R * p, where "*" is matrix multiplication, and q is
        considered as a 4 by 1 matrix.
        
        '''

        w = self.w
        x = self.x
        y = self.y
        z = self.z

        M = np.asarray((
            ( w, -x, -y, -z ),
            ( x,  w,  z, -y ),
            ( y, -z,  w,  x ),
            ( z,  y, -x,  w )
            ))
        
        return M

    def rotate_a_point(self, p):
        '''
        Rotate a single 3D point based on this quaternion. The input point is
        a numpy array of three element. 
        
        Internally, it is converted to a vector v = (0, x, y, z) and use the 
        following equation.
        
        v' = q x v x q*
        
        where x is quaternion multiplication.
        
        The implementation uses the following matrix-based method,
        
        q x v x p = (q x v) x p = p_R * q_L * v
                  = q x (v x p) = q_L * p_R * v
        
        '''
        
        q_L = self.to_left_matrix()
        qc_R = self.conjugate().to_right_matrix()
        
        v = np.asarray((0, p[0], p[1], p[2])).reshape((4, 1))
        
        r = np.matmul(qc_R, np.matmul(q_L, v))
        
        return np.asarray((r[1], r[2], r[3])).reshape(3)

    def rotate_points(self, ps):
        '''
        Rotate an array of points. The input points are a numpy 3-by-n matrix, 
        where each point is a column.
        
        '''
        
        q_L = self.to_left_matrix()
        qc_R = self.conjugate().to_right_matrix()
        
        vs = np.zeros((4, ps.shape[1]), ps.dtype)
        vs[1:4, :] = ps
        rs = np.matmul(qc_R, np.matmul(q_L, vs))
        
        return rs[1:4, :]


    

    # -------------- static methods --------------

    @staticmethod
    def angle_axis_to_quaternion(u):
        '''
        Convert an angle-axis representation of a 3D rotation to a unit 
        quaternion. This is essentially the exponential map.
        
        This function is only for internal usage. Please use the following 
        instead when it is needed to do this conversion.
        
            q = Quaternion.construct_from_angle_axis(u)
        
        See Joan Solà, "Quaternion Kinematics", section 2.4, eqation (101).
        
        '''
        
        angle = np.linalg.norm(u)
        
        if angle < EPSILON:
        
            w = 1.0
            x = 0.0
            y = 0.0
            z = 0.0
        
        else:
            
            u = u / angle
            
            w = cos(angle / 2)
            s = sin(angle / 2)
            
            x = u[0] * s
            y = u[1] * s
            z = u[2] * s
    
        return w, x, y, z

    @staticmethod
    def quaternion_to_angle_axis(w, x, y, z):
        '''
        Convert a unit quaternion representation of a 3D rotation to 
        angle-axis representation. This is essentially the logrithmic map.
        
        This function is only for internal usage. Please use the following 
        instead when it is needed to do this conversion.
        
            u = q.to_angle_axis(u)
        
        '''

        v_norm = sqrt(x * x + y * y + z * z)

        if v_norm < EPSILON:
            
            ux = 0
            uy = 0
            uz = 0

        else:
            
            angle = 2 * atan2(v_norm, w)
            
            ux = x * (angle / v_norm)
            uy = y * (angle / v_norm)
            uz = z * (angle / v_norm)
        
        u = np.asarray((ux, uy, uz))
        
        return u

    @staticmethod
    def rotation_matrix_to_quaternion(R):
        '''
        Convert a rotation matrix to a unit quaternion.
        
        This uses the Shepperd’s method for numerical stabilty.

        This function is only for internal usage. Please use the following 
        instead when it is needed to do this conversion.
        
            q = Quaternion.construct_from_rotation_matrix(R)

        '''
        
        # The rotation matrix must be orthonormal
        
        assert np.allclose(np.dot(R, R.conj().transpose()), np.eye(3), 
                           atol=EPSILON)
    
        # Check the determinant of R! It must be 1.
        assert math.isclose(np.linalg.det(R), 1, abs_tol=EPSILON)
        
        w2 = (1 + R[0, 0] + R[1, 1] + R[2, 2])
        x2 = (1 + R[0, 0] - R[1, 1] - R[2, 2])
        y2 = (1 - R[0, 0] + R[1, 1] - R[2, 2])
        z2 = (1 - R[0, 0] - R[1, 1] + R[2, 2])
            
        yz = (R[1, 2] + R[2, 1])
        xz = (R[2, 0] + R[0, 2])
        xy = (R[0, 1] + R[1, 0])
    
        wx = (R[2, 1] - R[1, 2])
        wy = (R[0, 2] - R[2, 0])
        wz = (R[1, 0] - R[0, 1])
                    
            
        if R[2, 2] < 0:
          
            if R[0, 0] > R[1, 1]:
            
                x = sqrt(x2)
                w = wx / x
                y = xy / x
                z = xz / x
            
            else:
                 
                y = sqrt(y2)
                w = wy / y
                x = xy / y
                z = yz / y
    
        else:
              
            if R[0, 0] < -R[1, 1]:
                 
                z = sqrt(z2)
                w = wz / z
                x = xz / z
                y = yz / z
            
            else:
                 
                w = sqrt(w2)
                x = wx / w
                y = wy / w
                z = wz / w
        
        w = w * 0.5
        x = x * 0.5
        y = y * 0.5
        z = z * 0.5
        
        return w, x, y, z

    @staticmethod
    def quaternion_to_rotation_matrix(w, x, y, z):
        '''
        Convert a unit quaternion to a rotation matrix.

        This function is only for internal usage. Please use the following 
        instead when it is needed to do this conversion.
        
            R = q.to_rotation_matrix()

        '''
        
        w2 = w * w
        x2 = x * x
        y2 = y * y
        z2 = z * z
        
        # Check the norm of the quaternion! It must be a unit quaternion!
        assert fabs(w2 + x2 + y2 + z2 - 1) < 1e-6
        
        wx = 2 * w * x
        wy = 2 * w * y
        wz = 2 * w * z
        
        xy = 2 * x * y
        xz = 2 * x * z
        yz = 2 * y * z
        
        R = np.asarray((
            ( w2 + x2 - y2 - z2,   xy - wz,             xz + wy           ),
            ( xy + wz,             w2 - x2 + y2 - z2,   yz - wx           ),
            ( xz - wy,             yz + wx,             w2 - x2 - y2 + z2 )
            ))
        
        return R


    # -------------------- class methods ------------------------
    
    @classmethod
    def identity(cls):
        '''
        Return the identity unit quaternion (1, 0, 0, 0).
        
        '''
        
        return cls(1, 0, 0, 0)
    
    @classmethod
    def construct_from_angle_axis(cls, u):
        '''
        Construct a quaternion from angle-axis representation of a rotation.
        
        This is essentially the exponential map.
        
        '''
        
        w, x, y, z = cls.angle_axis_to_quaternion(u)
        q = cls(w, x, y, z)
        q = q.normalize()
        
        return q

        
    @classmethod
    def construct_from_rotation_matrix(cls, R):
        '''
        Construct a quaternion from a rotation matrix.
        
        '''
        
        w, x, y, z = cls.rotation_matrix_to_quaternion(R)
        q = cls(w, x, y, z)
        
        
        return q

    
    @classmethod
    def interpolate(cls, q0, q1, t):
        '''
        Spherical linear interpolation (SLERP) given two orientations
        represented by the quaternion q0 and q1, and interpolation parameter
        t in [0, 1], such that the orientation will continuously change from
        q0 to q1 by rotating along a fixed axis when t changes.
        
        See Joan Solà, "Quanternion Kinematics", section 2.7.
        
        '''
        
        if t == 0:
            
            return q0
        
        elif t == 1:
            
            return q1
        
        elif t > 0 and t < 1:
            
        
            delta_q = q0.conjugate().multiply_quaternion(q1)
            
            u = delta_q.to_angle_axis()
            
            qut = cls.construct_from_angle_axis(u * t)
            
            qt = q0.multiply_quaternion(qut)
            
            return qt

        else:
        
            err_str = 'Interpolation parameter t must be in [0, 1], ' \
                'while t = %f !' % t
                
            raise ValueError(err_str)
        
        
    
    
    @classmethod
    def differentiate_local(cls, q1, q2, timestep):
        '''
        Given two quaternions representing two poses q1 and q2, and assuming q2
        is derived by a rotation from the first psoe q1 during a timestep,
        this method returns the angular speed (i.e., omega) of the local
        reference frame respect to pose q1.
        
        This is exactly the inverse of integrate_local().
        '''
        
        delta_q = q1.conjugate().multiply_quaternion(q2)
        
        u = delta_q.to_angle_axis()
        
        omega = u / timestep
        
        return omega
    
    @classmethod
    def integrate_local(cls, q, omega, timestep):
        '''
        Integrate instantanious angular velocity (i.e., omega) to the
        quaternion q during the timestep. The result is also a quaternion.
        
        Note that the angular velocity is measured relative to the pose
        represented by this quaternion, i.e., consider the current pose
        is this quaternion and omega is measured by a gyroscope. Anguler
        velocity is in rad/sec, and timestep is in sec.
        
        Note that we use zeroth-order backward integration. 
        
        See Joan Solà, "Quanternion Kinematics", section 4.6.
        '''
    
        u = omega * timestep
    
        delta_q = cls.construct_from_angle_axis(u)
        
        p = q.multiply_quaternion(delta_q)
        
        return p
    
if __name__ == '__main__':
    
    q0 = Quaternion(1, 0, 0, 0)
    
    q1 = Quaternion.construct_from_angle_axis(np.asarray((0, 0, 1)))
    
    p = Quaternion.interpolate(q0, q1, 0.5)
    
    print(p)
    
    
    
    
    
    
    
    
    
    
