import rclpy
from rclpy.node import Node
import os
from sensor_msgs.msg import PointCloud2, PointField
# from sensor_msgs_py.point_cloud2 import read_points_numpy, read_points
import struct
import ros2_numpy as rnpy
import math
import sys
import numpy as np

_DATATYPES = {}
_DATATYPES[PointField.INT8]    = ('b', 1)
_DATATYPES[PointField.UINT8]   = ('B', 1)
_DATATYPES[PointField.INT16]   = ('h', 2)
_DATATYPES[PointField.UINT16]  = ('H', 2)
_DATATYPES[PointField.INT32]   = ('i', 4)
_DATATYPES[PointField.UINT32]  = ('I', 4)
_DATATYPES[PointField.FLOAT32] = ('f', 4)
_DATATYPES[PointField.FLOAT64] = ('d', 8)

def read_points(cloud, field_names=None, skip_nans=False, uvs=[]):
    """
    Read points from a L{sensor_msgs.PointCloud2} message.
    @param cloud: The point cloud to read from.
    @type  cloud: L{sensor_msgs.PointCloud2}
    @param field_names: The names of fields to read. If None, read all fields. [default: None]
    @type  field_names: iterable
    @param skip_nans: If True, then don't return any point with a NaN value.
    @type  skip_nans: bool [default: False]
    @param uvs: If specified, then only return the points at the given coordinates. [default: empty list]
    @type  uvs: iterable
    @return: Generator which yields a list of values for each point.
    @rtype:  generator
    """
    assert isinstance(cloud, PointCloud2), 'cloud is not a sensor_msgs.msg.PointCloud2'
    fmt = _get_struct_fmt(cloud.is_bigendian, cloud.fields, field_names)
    width, height, point_step, row_step, data, isnan = cloud.width, cloud.height, cloud.point_step, cloud.row_step, cloud.data, math.isnan
    unpack_from = struct.Struct(fmt).unpack_from

    if skip_nans:
        if uvs:
            for u, v in uvs:
                p = unpack_from(data, (row_step * v) + (point_step * u))
                has_nan = False
                for pv in p:
                    if isnan(pv):
                        has_nan = True
                        break
                if not has_nan:
                    yield p
        else:
            for v in range(height):
                offset = row_step * v
                for u in range(width):
                    p = unpack_from(data, offset)
                    has_nan = False
                    for pv in p:
                        if isnan(pv):
                            has_nan = True
                            break
                    if not has_nan:
                        yield p
                    offset += point_step
    else:
        if uvs:
            for u, v in uvs:
                yield unpack_from(data, (row_step * v) + (point_step * u))
        else:
            for v in range(height):
                offset = row_step * v
                for u in range(width):
                    yield unpack_from(data, offset)
                    offset += point_step
def _get_struct_fmt(is_bigendian, fields, field_names=None):
    fmt = '>' if is_bigendian else '<'

    offset = 0
    for field in (f for f in sorted(fields, key=lambda f: f.offset) if field_names is None or f.name in field_names):
        if offset < field.offset:
            fmt += 'x' * (field.offset - offset)
            offset = field.offset
        if field.datatype not in _DATATYPES:
            print('Skipping unknown PointField datatype [%d]' % field.datatype, file=sys.stderr)
        else:
            datatype_fmt, datatype_length = _DATATYPES[field.datatype]
            fmt    += field.count * datatype_fmt
            offset += field.count * datatype_length

    return fmt

class ObjectDepth(Node):

    def __init__(self, gear_c: tuple):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            PointCloud2,
            '/camera/depth/color/points',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.gx = gear_c[0]
        self.gy = gear_c[1]
        self.dist_x = None
        self.dist_y = None
        self.dist_z = None

    def listener_callback(self, msg : PointCloud2):
        # data = read_points_numpy(msg,field_names = ("x", "y", "z"), skip_nans=True, uvs=[[334,349]])
        data = read_points(msg, skip_nans=False, uvs=[[self.gx, self.gy]])
        # print(data.shape)
        # for i in range(0,15,3):
        #     print(f"measurement for ({self.gx+i+40},{self.gy+i+40}): " + str(data[self.gx+i+40][self.gy+i+40]))
        # print("\n"*5)
        for i in data:
            self.dist_x = i[0]
            self.dist_y = i[1]
            self.dist_z = i[2]
        # for i in range(5):
        #     for j in range(5):
        #         print(f"measurement for ({self.gx-2+i},{self.gy-2+j}): " + str(data[self.gx-2+i][self.gy-2+j]))
        #     print("\n")
        
        # print("Data for central point: ",str(data[(self.gx, self.gy)]))
        # orientations = []
        # orientations.append(data[self.gx][self.gy])
        # orientations.append(data[848-self.gx][self.gy])
        # orientations.append(data[self.gx][480-self.gy])
        # orientations.append(data[848-self.gx][480-self.gy])
        # print("Orientations:\n"+"\n".join([str(o) for o in orientations]),end="\n\n")
        # print("\n\n".join(["Distance away from camera: "+str(__import__("math").sqrt(sum([i**2 for i in d])))+" meters" for d in orientations]))
        # data = read_points(msg,field_names = ("x", "y", "z"), uvs=[437])
        # self.get_logger().info("Data: "+ ", ".join([str(d) for d in data]))