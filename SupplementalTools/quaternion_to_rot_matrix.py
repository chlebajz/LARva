def quaternion_rotation_matrix(Q):
    """
    Covert a quaternion into a full three-dimensional rotation matrix.
    source: https://automaticaddison.com/how-to-convert-a-quaternion-to-a-rotation-matrix/
    We are not authors of this function.

    Input
    :param Q: A 4 element array representing the quaternion (q0,q1,q2,q3)

    Output
    :return: A 3x3 element matrix representing the full 3D rotation matrix.
             This rotation matrix converts a point in the local reference
             frame to a point in the global reference frame.
    """
    # Extract the values from Q
    q0 = Q[0]
    q1 = Q[1]
    q2 = Q[2]
    q3 = Q[3]

    # First row of the rotation matrix
    r00 = 2 * (q0 * q0 + q1 * q1) - 1
    r01 = 2 * (q1 * q2 - q0 * q3)
    r02 = 2 * (q1 * q3 + q0 * q2)

    # Second row of the rotation matrix
    r10 = 2 * (q1 * q2 + q0 * q3)
    r11 = 2 * (q0 * q0 + q2 * q2) - 1
    r12 = 2 * (q2 * q3 - q0 * q1)

    # Third row of the rotation matrix
    r20 = 2 * (q1 * q3 - q0 * q2)
    r21 = 2 * (q2 * q3 + q0 * q1)
    r22 = 2 * (q0 * q0 + q3 * q3) - 1

    # 3x3 rotation matrix
    rot_matrix = [[r00, r01, r02], [r10, r11, r12], [r20, r21, r22]]
    return rot_matrix

if __name__ == "__main__":
    rotation_matrix = quaternion_rotation_matrix([-0.500, 0.500, -0.500, 0.500])
    T = [rotation_matrix[i][:] for i in range(len(rotation_matrix))]
    x, y, z = -0.087, 0.013, 0.287 #data from the robot
    T[0].append(x)
    T[1].append(y)
    T[2].append(z)
    T.append([0, 0, 0, 1])
    print(T)