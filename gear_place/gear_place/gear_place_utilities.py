X_OFFSET = 0.038  # offset from the camera to the gripper
Y_OFFSET = 0.03
Z_TO_TABLE = -0.247
Z_CAMERA_OFFSET = 0.0435
X_DEPTH_TO_COLOR = [(467,389),(402,347),(376,331),(405,349),(422,360)] # points made using depth values and color values
Y_DEPTH_TO_COLOR = [(148,177),(191,204),(103,146),(300,275),(241,238)]
from math import sqrt, sin, cos, pi

def norm(x: float, y: float, z: float) -> float:
  return __import__("math").sqrt(x**2 + y**2 + z**2)

def avg(arr : list) -> float:
    return sum(arr)/len(arr)

def dist_from_point(x_val : float, y_val : float)->float:
    return sqrt(x_val**2+y_val**2)

def distance_between_two_points(x_vals : list, y_vals : list) -> float:
    return abs(dist_from_point(x_vals[1],y_vals[1])-dist_from_point(x_vals[0],y_vals[0]))

def rotate_points_around_angle(x_val : float, y_val : float, angle : float) -> tuple:
    return x_val * cos(angle) - y_val * sin(angle),x_val * sin(angle) + y_val * cos(angle)

def convert_color_to_depth(point : tuple) -> tuple:
    """
    predicts the corresponding depth point from a given color point using multiple linear functions
    built from already found points
    """
    estimated_x_vals = [(X_DEPTH_TO_COLOR[j][1]-X_DEPTH_TO_COLOR[i][1])
                        /(X_DEPTH_TO_COLOR[j][0]-X_DEPTH_TO_COLOR[i][0])
                        *(point[0]-X_DEPTH_TO_COLOR[i][0])+X_DEPTH_TO_COLOR[i][1]
                        for i in range(len(X_DEPTH_TO_COLOR)) for j in range(len(X_DEPTH_TO_COLOR))
                        if i != j]
    estimated_y_vals = [(Y_DEPTH_TO_COLOR[j][1]-Y_DEPTH_TO_COLOR[i][1])
                        /(Y_DEPTH_TO_COLOR[j][0]-Y_DEPTH_TO_COLOR[i][0])
                        *(point[1]-Y_DEPTH_TO_COLOR[i][0])+Y_DEPTH_TO_COLOR[i][1]
                        for i in range(len(Y_DEPTH_TO_COLOR)) for j in range(len(Y_DEPTH_TO_COLOR))
                        if i != j]

    return (int(round(sum(estimated_x_vals)/len(estimated_x_vals),0)),int(round(sum(estimated_y_vals)/len(estimated_y_vals),0)))

def convert_color_to_depth_radius(radius : float) -> float:
    return 0.735831781 * radius

def average_of_points(self, arr : list)-> tuple:
    """
    Takes in a list of points and returns the average of them
    """
    num_points = len(arr)
    return (
        sum([arr[i][0] for i in range(num_points)])/num_points,
        sum([arr[i][1] for i in range(num_points)])/num_points,
        min([arr[i][2] for i in range(num_points)])
    )

def closest_to_center(self, arr : list) -> int:
    """
    Returns the coordinates of the gear which is closest to the center of the camera
    """
    vals = [sqrt(sum([(arr[i][j]-[X_OFFSET,Y_OFFSET][j])**2 for j in range(2)])) for i in range(len(arr))]
    try:
        return vals.index(min(vals))
    except:
        return -1

def remove_identical_points(self, arr : list, radius_vals : dict) -> list:
    """
    Removes duplicate coordinates from different positions
    """
    bad_measurements = []
    for i in range(len(arr) - 1):
        radius_list = []
        if radius_vals[arr[i]]!=0:
            radius_list.append(radius_vals[arr[i]])
        close_vals = []
        close_vals.append(arr[i])
        for j in range(i + 1, len(arr)):
            
            if (
                sqrt((arr[i][0] - arr[j][0]) ** 2 + (arr[i][1] - arr[j][1]) ** 2)
                <= 0.03
            ):  # Gets rid of the points which are within 30mm of each other
                bad_measurements.append(
                    j
                )  # ensures that the first instance of a valid gear is saved

            if (sqrt(sum([(arr[i][k]-arr[j][k])**2 for k in range(3)]))<=0.01): # adds point to list if it is close enough
                close_vals.append(arr[j])
                if radius_vals[(arr[j])]!=0:
                    radius_list.append(radius_vals[arr[j]])
        arr[i] = self.average_of_points(close_vals)
        radius_vals[arr[i]] = avg(radius_list) if len(radius_list)>0 else 0
    bad_measurements = sorted(list(set(bad_measurements)))[
        ::-1
    ]  # sorts the indicies in decending order so the correct values are removed in next loop
    for ind in bad_measurements:  # deletes duplicated gears
        del arr[ind]
    return arr