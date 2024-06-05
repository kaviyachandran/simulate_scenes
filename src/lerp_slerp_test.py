import time
import rospy

import tf.transformations as tf
import numpy as np
from geometry_msgs.msg import Quaternion, TransformStamped


def interpolate(self, A, B, TOTAL_VAL=50):
    """
    A is a list with three elements
    B is a list with three elements
    """

    A = np.array(A)
    B = np.array(B)

    # Create a linear interpolation factor array using np.linspace
    c = np.linspace(0.0, 1.0, TOTAL_VAL)

    x = ((1 - c) * A[0]) + c * B[0]
    y = ((1 - c) * A[1]) + c * B[1]
    z = ((1 - c) * A[2]) + c * B[2]
    return x.tolist(), y.tolist(), z.tolist()


def quaternion_slerp( q_init, q_final, TOTAL_VAL=10):
    """
    q_init and q_final is a list with order x,y,z,w
    return list with [Quaternion()]
    """
    ratio_between_qs = np.linspace(0, 1.0, TOTAL_VAL).tolist()
    q_list = []

    for _, val in enumerate(ratio_between_qs):
        print("fraction ", val)
        q_next = tf.quaternion_slerp(q_init, q_final, val)

        q = Quaternion()
        q.x = q_next[0]
        q.y = q_next[1]
        q.z = q_next[2]
        q.w = q_next[3]
        q_list.append(q)

    return q_list


def interpolate_3D_position(self, A, B, TOTAL_VAL=50):
    """
    A is a list with three elements
    B is a list with three elements
    return three list: x, y, z with length of TOTAL_VAL
    """
    x = []
    y = []
    z = []
    j = 1.0
    c = 0.0
    for i in range(TOTAL_VAL):
        c = j / TOTAL_VAL
        x_val = ((1 - c) * A[0]) + c * B[0]
        y_val = ((1 - c) * A[1]) + c * B[1]
        z_val = ((1 - c) * A[2]) + c * B[2]
        x.append(x_val)
        y.append(y_val)
        z.append(z_val)
        j = j + 1

    return x, y, z


def interpolate_with_quaternion(A, B, quat, TOTAL_VAL=50):
    A = np.around(np.asarray(A).astype(np.double), 5).tolist()
    try:
        B = np.around(np.asarray(B).astype(np.double), 5).tolist()
    except:
        raise ValueError("B is not a list")
    x, y, z = interpolate(A, B, TOTAL_VAL)
    hand_waypoints = [TransformStamped()]
    del hand_waypoints[-1]

    for i in range(len(x)):
        waypoint = TransformStamped()
        waypoint.header.frame_id = "world"
        waypoint.child_frame_id = "meh"
        waypoint.transform.rotation = quat[i]
        waypoint.transform.translation.x = x[i]
        waypoint.transform.translation.y = y[i]
        waypoint.transform.translation.z = z[i]
        waypoint.header.stamp = time.time()
        hand_waypoints.append(waypoint)

    return hand_waypoints


if __name__ == "__main__":
    num_waypoints = 50
    q_current = [0.707, 0.707, 0.0, 0.0]
    q_final = [0.0, 0.0, 0.0, 1.0]
    q_slerp_A_B = quaternion_slerp(q_current, q_final, num_waypoints)
    waypoints = interpolate_with_quaternion(A=[0.0, 0.0, 0.0], B=[1.0, 1.0, 1.0], quat=q_slerp_A_B,
                                            TOTAL_VAL=num_waypoints)
