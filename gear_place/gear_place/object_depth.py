from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
import struct
import math
import sys

_DATATYPES = {}
_DATATYPES[PointField.INT8] = ("b", 1)
_DATATYPES[PointField.UINT8] = ("B", 1)
_DATATYPES[PointField.INT16] = ("h", 2)
_DATATYPES[PointField.UINT16] = ("H", 2)
_DATATYPES[PointField.INT32] = ("i", 4)
_DATATYPES[PointField.UINT32] = ("I", 4)
_DATATYPES[PointField.FLOAT32] = ("f", 4)
_DATATYPES[PointField.FLOAT64] = ("d", 8)

def dist_between_points(points : list):
    return __import__("math").sqrt(sum([(points[0][i]-points[1][i])**2 for i in range(2)]))

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
    assert isinstance(cloud, PointCloud2), "cloud is not a sensor_msgs.msg.PointCloud2"
    fmt = _get_struct_fmt(cloud.is_bigendian, cloud.fields, field_names)
    width, height, point_step, row_step, data, isnan = (
        cloud.width,
        cloud.height,
        cloud.point_step,
        cloud.row_step,
        cloud.data,
        math.isnan,
    )
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
                try:
                    yield unpack_from(data, (row_step * v) + (point_step * u))
                except:
                    continue
        else:
            for v in range(height):
                offset = row_step * v
                for u in range(width):
                    yield unpack_from(data, offset)
                    offset += point_step


def _get_struct_fmt(is_bigendian, fields, field_names=None):
    fmt = ">" if is_bigendian else "<"

    offset = 0
    for field in (
        f
        for f in sorted(fields, key=lambda f: f.offset)
        if field_names is None or f.name in field_names
    ):
        if offset < field.offset:
            fmt += "x" * (field.offset - offset)
            offset = field.offset
        if field.datatype not in _DATATYPES:
            print(
                "Skipping unknown PointField datatype [%d]" % field.datatype,
                file=sys.stderr,
            )
        else:
            datatype_fmt, datatype_length = _DATATYPES[field.datatype]
            fmt += field.count * datatype_fmt
            offset += field.count * datatype_length

    return fmt


class ObjectDepth(Node):
    def __init__(self, points:list, width_vals:dict):
        """
        Takes in the gear center (or any other pixel) and finds the distance from the camera to that point
        """
        super().__init__("minimal_subscriber")
        self.subscription = self.create_subscription(
            PointCloud2, "/camera/depth/color/points", self.listener_callback, 10
        )
        self.subscription  # prevent unused variable warning
        self.points = points
        self.radius_coordinates = width_vals
        self.coordinates = []
        self.radius_vals = {}

    def listener_callback(self, msg: PointCloud2):
        """
        Reads in the point cloud and returns the distance away from the given point.
        Used for finding the distance from the camera to the center of the gear
        """
        for point in self.points:
            radius_point = [0 for _ in range(3)]
            try:
                data = read_points(msg, skip_nans=False, uvs=[point])
            except:
                self.get_logger().error("Buffer out of bounds for gear center")
                return
            if self.radius_coordinates!={}:
                distances = []
                for val in self.radius_coordinates[point]:
                    try:
                        measurement = read_points(msg, skip_nans=False, uvs = [val])
                    except:
                        self.get_logger().error("Buffer out of bounds for gear edge")
                        measurement = []
                    for i in measurement:
                        if i.count(0.0) == 0 and i.count(None)==0:
                            radius_point = i
                for i in data:
                    if radius_point.count(0.0)!=3:
                        distances.append(dist_between_points([i,radius_point]))
                    else:
                        distances.append(0)
                    self.coordinates.append((i[0],i[1],i[2]))
                self.radius_vals[(i[0],i[1],i[2])] = (min([val for val in distances if val!=0]) if distances.count(0)!= len(distances) else 0)
                # the radius value is 0 if a radius can not be found
            else:
                for i in data:
                    self.coordinates.append((i[0],i[1],i[2]))


