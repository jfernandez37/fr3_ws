import PyKDL
import numpy as np
import math
from geometry_msgs.msg import Pose, Quaternion, TransformStamped

from builtin_interfaces.msg import Time


def multiply_pose(p1: Pose, p2: Pose) -> Pose:
    """
    Use KDL to multiply two poses together.
    Args:
        p1 (Pose): Pose of the first frame
        p2 (Pose): Pose of the second frame
    Returns:
        Pose: Pose of the resulting frame
    """

    o1 = p1.orientation
    frame1 = PyKDL.Frame(
        PyKDL.Rotation.Quaternion(o1.x, o1.y, o1.z, o1.w),
        PyKDL.Vector(p1.position.x, p1.position.y, p1.position.z),
    )

    o2 = p2.orientation
    frame2 = PyKDL.Frame(
        PyKDL.Rotation.Quaternion(o2.x, o2.y, o2.z, o2.w),
        PyKDL.Vector(p2.position.x, p2.position.y, p2.position.z),
    )

    frame3 = frame1 * frame2

    # return the resulting pose from frame3
    pose = Pose()
    pose.position.x = frame3.p.x()
    pose.position.y = frame3.p.y()
    pose.position.z = frame3.p.z()

    q = frame3.M.GetQuaternion()
    pose.orientation.x = q[0]
    pose.orientation.y = q[1]
    pose.orientation.z = q[2]
    pose.orientation.w = q[3]

    return pose


def convert_transform_to_pose(t: TransformStamped) -> Pose:
    p = Pose()

    p.position.x = t.transform.translation.x
    p.position.y = t.transform.translation.y
    p.position.z = t.transform.translation.z
    p.orientation = t.transform.rotation

    return p


def euler_from_quaternion(quaternion: Quaternion):
    """
    Converts quaternion (w in last place) to euler roll, pitch, yaw
    quaternion = [x, y, z, w]
    """
    x = quaternion.x
    y = quaternion.y
    z = quaternion.z
    w = quaternion.w

    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (w * y - z * x)
    pitch = np.arcsin(sinp)

    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


def quaternion_from_euler(roll: float, pitch: float, yaw: float) -> Quaternion:
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    q = [0] * 4
    q[0] = cy * cp * cr + sy * sp * sr
    q[1] = cy * cp * sr - sy * sp * cr
    q[2] = sy * cp * sr + cy * sp * cr
    q[3] = sy * cp * cr - cy * sp * sr

    q_msg = Quaternion()
    q_msg.w = q[0]
    q_msg.x = q[1]
    q_msg.y = q[2]
    q_msg.z = q[3]

    return q_msg


def transform_from_pose(
    pose: Pose, frame_id: str, parent: str, stamp: Time
) -> TransformStamped:
    t = TransformStamped()

    t.header.stamp = stamp
    t.header.frame_id = parent
    t.child_frame_id = frame_id

    t.transform.translation.x = pose.position.x
    t.transform.translation.y = pose.position.y
    t.transform.translation.z = pose.position.z
    t.transform.rotation = pose.orientation

    return t


def build_pose(
    x: float, y: float, z: float, roll: float, pitch: float, yaw: float
) -> Pose:
    p = Pose()
    p.position.x = x
    p.position.y = y
    p.position.z = z

    p.orientation = quaternion_from_euler(roll, pitch, yaw)

    return p
