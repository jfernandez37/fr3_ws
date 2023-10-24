# gear_place_classes.py

## init
```python 
self.current_camera_angle = 0.0
```
This is able to be updated through the `get_camera_angle` service. This is so the `move_cartesian_angle` service has the current camera angle and can move in relation to the camera and not the base.

```python 
self.current_joint_positions = []
```
This gets the current joint posiitons using the `get_joint_positions` service. This allows the python program to save a position and return back to that position after movements are made.

```python 
self.name_of_service_client = self.create_client(NameOfService,"name_of_service")
```
This creates the clients for each service so that they can be called in the python program. The services are all defined in `robot_commander.cpp`

## _call_move_to_named_pose_service

```python
request = MoveToNamedPose.Request()

request.pose = named_pose
```
This function starts by adding the name of the pose to the request

```python
future = self.move_to_named_pose_client.call_async(request)

rclpy.spin_until_future_complete(self, future, timeout_sec=10)

if not future.done():
    raise Error("Timeout reached when calling move_to_named_pose service")
```
The request is then made and given a timeout of 10 seconds to finish. If it is not finished, an error message will output and the program will stop.

```python
result: MoveToNamedPose.Response
result = future.result()

if not result.success:
    self.get_logger().error(f"Unable to move to pose: {named_pose}")
    raise Error("Unable to move to pose")
```
The result is then retrieved from the future and if the request was not successful, an error is raised.

## _call_move_cartesian_service

This function is essentially the same as `_call_move_to_named_pose_service` except the request and response is changed from the `MoveToNamedPose` service to `MoveCartesian`.

```python
request = MoveCartesian.Request()

request.x = x
request.y = y
request.z = z
request.max_velocity = v_max
request.acceleration = acc

future = self.move_cartesian_client.call_async(request)
```
This is the request. The variables in the request are changed to the ones required for MoveCartesian and the name of the client is changed to the  `move_cartesian_client`.

## _call_move_cartesian_angle_service
This function is the same as `_call_move_cartesian_service`, but with an additional variable in the request for the angle. This is the request:

```python
request = MoveCartesianAngle.Request()

request.x = x
request.y = y
request.z = z
request.max_velocity = v_max
request.acceleration = acc
request.angle = angle
```

## _call_pick_up_gear_service
This function starts by making a list containing three zeroes:
```python
gear_center_target = [0 for _ in range(3)]
```
This is done so the program can loop until a valid movement is found for the gear.

Then, the primary loop is called
```python
while (
    gear_center_target.count(0) == 3 or None in gear_center_target
):  # runs until valid coordinates are found
```
This loop runs until valid coordinates are found from `ObjectDepth`.

To find the gear, an instance of the `FindObjectColor` class is made. This class is then spun using rclpy.
```python
find_object_color = FindObjectColor()
rclpy.spin_once(find_object_color) # Finds the gear
```

Until a gear is found, attempts are continued. Every five unsuccessful attempts, the robot moves a small amount to one side and up the table.
```
c = 0
while (
    find_object_color.ret_cent_gear().count(None) != 0
):  # Runs and guarantees that none of the coordinates are none type
    c += 1

    if c % 5 == 0:
        self._call_move_cartesian_smooth_service(
            0.05, 0.05 * (-1 if c % 2 == 1 else 1), 0.0, 0.15, 0.2
        )  # Moves to the center of the cart
        sleep(1)
    else:
        find_object_color.destroy_node()
        find_object_color = FindObjectColor()
        rclpy.spin_once(find_object_color)
```
The counter is used so that the robot can move every five attempts. Each time, the previous scan is destroyed and a new one is created and spun.

After the center of the gear is found, the distance from the center of the camera is found using the `ObjectDepth` class. This is attempted once and if it is not found, the main while loop will run again, restarting the scan.
```python
object_depth = ObjectDepth([convert_color_to_depth(find_object_color.ret_cent_gear())],{})
rclpy.spin_once(object_depth)  # Gets the distance from the camera
object_depth.destroy_node()  # Destroys the node to avoid errors on next loop
find_object_color.destroy_node()
gear_center_target = object_depth.coordinates[0]
```

If the coordinates are successfully found for the center of the gear, a request is made and `pick_up_gear_client` is used to call the service.
```python
request = PickUpGear.Request()

request.x = -1 * gear_center_target[1] + X_OFFSET
request.y = -1 * gear_center_target[0] + Y_OFFSET
request.z = -1 * gear_center_target[2] + Z_CAMERA_OFFSET + 0.0075
request.object_width = object_width

future = self.pick_up_gear_client.call_async(request)
```

The rest of the function is identical to the previous ones.

## _call_put_gear_down_service
The request for this service only has one variable, the z movement. This is the simplest way to put the gear down, but it requires the z movement to already be known. This is the request:
```python
request = PutGearDown.Request()
z_movement = max(
    Z_TO_TABLE, z + Z_CAMERA_OFFSET
)  + 0.0005 # z distance from current position to the gear and makes sure it does not try to go below the table
request.z = z_movement
```

## _call_move_above_gear
First, an instance of the `MovingGear` class is run until the gear is found successfuly twice. The class only adds values to x_vals if the gear is found twice
```python
moving_gear = MovingGear()
while not moving_gear.found_gear or len(moving_gear.x_vals) == 0:
    moving_gear.run()
```

Then, the velocity and acceleration of the movements for the robot are declared. This is so the interception time can accurately be calculated.
```python
velocity = 0.15
acceleration = 0.2
```

The `MovingGear` class calculates the slope and intercept of the linear equation of the moving gear, so those are recieved by using this line:
```python
slope, intercept = moving_gear.distance_formula()
```

After this, the intersection time of when the robot and the gear will intersect is calculated using algebra.
```python
intersection_time = (
        -(velocity**2) / acceleration - intercept
    ) / (
        slope - velocity
    )
```

Then, using the `MovingGear` classs, the distance of the gear at the intersection time is found:
```python
distance_at_intersection = moving_gear.distance_to_point(
    moving_gear.point_from_time(intersection_time)
)
```

Then, if the maximum velocity of the robot will have to be changed, this is accounted for:
```python
if (
    velocity**2
) / acceleration > distance_at_intersection:  # runs if max velocity needs to be lowered in move_cartesian
    velocity = (
        distance_at_intersection / (velocity / acceleration) * 0.9
    )  # lowers the velocity
    intersection_time = (
        -(velocity**2) / acceleration
        - intercept
    ) / (
        slope - velocity
    )
```

The movement values for the intersection time are then calculated for the robot using the `MovingGear` class.
```python
x_value, y_value = moving_gear.point_from_time(intersection_time)
```

Since the acceleration can not be 1, it means that there will be distance lost to accelration. This means that it needs to be made up. To do this, first, the ratio of x to the total movement and y to the total movement must be calculated.
```python
ratio_x = x_value/(x_value+y_value)
ratio_y = 1 - ratio_x
```

Then, the distance lost to accelration is found.
```python
second_acceleration = 0.3
second_t1 = gear_speed/second_acceleration

distance_lost_to_acc = gear_speed * second_t1 / 2
```

The final movements are then calculated and the speed at which the gear is moving are found next.
```python
final_x, final_y = moving_gear.point_from_time(intersection_time*3)
gear_speed = (distance_between_two_points(moving_gear.x_vals, moving_gear.y_vals)) / (moving_gear.times[1]-moving_gear.times[0])
```

Finally, everything is put together for the movements required for the gripper to be over the gear.
```python
self._call_move_cartesian_angle_service(y_value * -1 + X_OFFSET + ratio_y * distance_lost_to_acc,x_value * -1 + Y_OFFSET + ratio_x * distance_lost_to_acc,0.0,velocity, acceleration, self.current_camera_angle)
self._call_move_cartesian_angle_service(final_y * -1 + X_OFFSET,final_x * -1 + Y_OFFSET, 0.0, gear_speed, second_acceleration,self.current_camera_angle)
```
The angle movement has to be used because all calculations for x and y values where done in relation to the camera and not the base.

## _call_pick_up_moving_gear_service
Largely, the math in this section is the same as `_call_move_above_gear`. The major difference is that this function moves down to pick the gear up instead of moving above the gear.

## average_of_points
This function takes in a list of points and finds the average of all of them. First, the number of points is found.
```python
num_points = len(arr)
```
Then, a tuple is returned with the average of all of the points as a single point.
```python
return (
    sum([arr[i][0] for i in range(num_points)])/num_points,
    sum([arr[i][1] for i in range(num_points)])/num_points,
    min([arr[i][2] for i in range(num_points)])
)
```

## closest_to_center
This finds the gear closest to the center of the gripper. First, the distances from the center of the gripper and the center of the gear is taken.
```python
vals = [sqrt(sum([(arr[i][j]-[X_OFFSET,Y_OFFSET][j])**2 for j in range(2)])) for i in range(len(arr))]
```
Then, the index of the minimum value is returned. If there is an issue, -1 is returned.
```python
try:
    return vals.index(min(vals))
except:
    return -1
```

## remove_identical_points
This function removes points which are identical from the list. This can happen when scans are done more than once or scans are done in different positions in relation a primary position. First, an empty list is created to hold all of the bad_measurements and a loop is started that loops through all of points.
```python
bad_measurements = []
for i in range(len(arr) - 1):
```
Inside the loop, a list is created containing the radius values for that point. The radius values are recorded to detect the size of the gear.
```python
radius_list = []
if radius_vals[arr[i]]!=0:
    radius_list.append(radius_vals[arr[i]])
```
Then, a list which records all of the points closest to the current point. This is done so that the average of all points can be saved and accuracy is increased.
```python
close_vals = []
close_vals.append(arr[i])
```
Then every point after the current one is looped through. If it found to be a duplicate, its index is added to the bad_measurements list.
```python
for j in range(i + 1, len(arr)):
              
    if (
        sqrt((arr[i][0] - arr[j][0]) ** 2 + (arr[i][1] - arr[j][1]) ** 2)
        <= 0.03
    ):  # Gets rid of the points which are within 30mm of each other
        bad_measurements.append(
            j
        )  # ensures that the first instance of a valid gear is saved
```
Then, if the point is close enough, it is added to the `close_vals` list and `radius_list` so that the average can be taken of them.
```python
if (sqrt(sum([(arr[i][k]-arr[j][k])**2 for k in range(3)]))<=0.01): # adds point to list if it is close enough
    close_vals.append(arr[j])
    if radius_vals[(arr[j])]!=0:
    radius_list.append(radius_vals[arr[j]])
```
The current point is then set to the average of all close points and the radius vals for that point is set to the average of all close radii.
```python
arr[i] = self.average_of_points(close_vals)
radius_vals[arr[i]] = avg(radius_list) if len(radius_list)>0 else 0
```
After this, the list of bad measurements has its duplicates deleted, it is sorted, then reversed. The list is then looped through, removing any index. It has to be reversed because if not, indexes will be changed and incorrect values will be deleted. Finally, the resulting array of points is returned.
```python
bad_measurements = sorted(list(set(bad_measurements)))[
    ::-1
]  # sorts the indicies in decending order so the correct values are removed in next loop
for ind in bad_measurements:  # deletes duplicated gears
    del arr[ind]
return arr
```

#find_distance
This function finds the distance from (0,0) and a point. This is used to invalidate points too far away from a position.
```python
return sqrt(arr[0] ** 2 + arr[1] ** 2)
```

## _call_pick_up_multiple_gears
First, four lists are made. The first holds the distances from the centers of the gears found to the home position. The second is a list of the movements to get to the next scanning position. The final two are the x movments and the y movments of all movements to scanning position. This is used to hold the accurate distances to the gears from the home position. Also, a counter for the number of gears found and a dictionary to hold updated radius values is created.
```python
distances_from_home = []
robot_moves = [
    [0.0, -0.1],
    [-0.1, 0.0],
    [-0.1, 0.0],
    [0.0, 0.1],
    [0.1, 0.0],
    [0.1, 0.1],
    [-0.1, 0.0],
    [-0.1, 0.0],
]  # cartesian movements starting at home position. Scans the area in front of the robot.
x_movements = [a[0] for a in robot_moves]  # just the x direction movements
y_movements = [a[1] for a in robot_moves]  # just the y direction movements
gears_found = 0
updated_radius_vals = {}
```
Next, inside of a loop that runs until gears are found, the moves are looped through. 
```python
while gears_found == 0:
    for ind in range(len(robot_moves)+1):  # loops through the scanning positions
```
For testing purposes, the `_call_get_joint_positions` and `_call_get_camera_angle` functions are run and their results are outputed to the terminal.
```python
self._call_get_joint_positions()
self.get_logger().info("Current joint positions: " + ", ".join([str(val) for val in self.current_joint_positions]))
self._call_get_camera_angle()
self.get_logger().info(f"Current camera angle in radians: {self.current_camera_angle}")
```
For each position, two scans are run. For each scan, the `MultipleGears` created and spun, the `ObjectDepth` class is created and run, and the distances and radii are saved to the lists. These values are in relation to the home position.
```python
for _ in range(2):  # runs until nothing is found, while something is found but coordinates are not, or if it runs 5 times with no results
    multiple_gears = MultipleGears(self.connected)
    rclpy.spin_once(multiple_gears)  # finds multiple gears if there are multiple
    self.connected = multiple_gears.connected
    while (
        sum([cent.count(None) for cent in multiple_gears.g_centers]) != 0
        or not multiple_gears.ran
    ):  # loops until it has run and until there are no None values
        multiple_gears.destroy_node()
        multiple_gears = MultipleGears(self.connected)
        rclpy.spin_once(multiple_gears)
        self.connected = multiple_gears.connected
    object_depth = ObjectDepth(multiple_gears.g_centers, multiple_gears.dist_points)
    rclpy.spin_once(object_depth)  # Gets the distance from the camera
    object_depth.destroy_node()  # Destroys the node to avoid errors on next loop
    for coord in object_depth.coordinates:
        if coord.count(0.0)==0:  # adds coordinates if not all 0. Duplicates are removed later
            distances_from_home.append(
                (
                    -1 * coord[1] + sum(x_movements[:ind]),
                    -1 * coord[0] + sum(y_movements[:ind]),
                    -1 * coord[2],
                )
            )
            updated_radius_vals[(
                    -1 * coord[1] + sum(x_movements[:ind]),
                    -1 * coord[0] + sum(y_movements[:ind]),
                    -1 * coord[2],
                )] = object_depth.radius_vals[coord]
    multiple_gears.destroy_node()
```
Finally, the robot moves to the next scanning position.
```python
if ind != len(robot_moves):
    self._call_move_cartesian_smooth_service(
        robot_moves[ind][0], robot_moves[ind][1], 0.0, 0.15, 0.2
    )  # moves to the next position
```
Duplicate points and points too far away from the home position are then removed from `distances_from_home`.
```python
distances_from_home = self.remove_identical_points(distances_from_home, updated_radius_vals)  # since gears will be repeated from different positions, repetitions are removed

distances_from_home = [
    distances_from_home[i]
    for i in range(len(distances_from_home))
    if self.find_distance(distances_from_home[i]) <= 0.4
]  # removes points which are too far from the home position
```
The `gears_found` variable is set. If it is 0, the scanning will restart. if not, the number of gears found will output and the movments will print to the screen.
```python
    gears_found = len(distances_from_home)
    if gears_found==0:
        self._call_move_to_named_pose_service("home")
        self.get_logger().info("No gears found. Trying again")
self.get_logger().info(
    f"{len(distances_from_home)} gears found. Picking up the gears"
)  # outputs the number of gears found
for movment in distances_from_home:
    self.get_logger().info("Movement: " + str(movment))
```
The robot then returns to the home position, and declares 4 variables. The first of these is a list which holds the last point visited. This is in relation to the home position and allows the robot to go from one gear to the next without having to return to the home position between each gear. Second is a boolean variable which keeps track of whether the offset is needed. It is only needed for the first time, as once the gripper is directly over the gear, the next movement does not require an offset. Finally, the last two are thresholds for the gear sizes. This allows the program to see which size the gear is. The gear is either small, medium, or large.
```python
self._call_move_to_named_pose_service("home")
last_point = [0, 0]
offset_needed = True
low_gear_threshold = 0.0275
high_gear_thershold = 0.041
```
Next, each movement is looped through. To start each loop, the move is calculated in relation to the current position instead of the home position and the current position is set.
```python
move = [
    gear_point[i] - last_point[i] for i in range(2)
]  # finds the next movement to the next gear
last_point = gear_point
```
The size of the gear is then found using the radius values and the threshold values declared earlier. If the radius value was not declared successfully, a message will be output to the screen. If it is found successfully, the color of the gear will be output. To classify the gear, its radius is put into a list with the thresholds which is then sorted. Depending on the position of the radius in the sorted list, the gear size can be determined.
```python
if updated_radius_vals[gear_point] ==0:
    gear_color == "not found"
    self.get_logger().info("Could not find gear color")
else:
    thresholds = sorted([low_gear_threshold, high_gear_thershold,updated_radius_vals[gear_point]])
    gear_color = ["yellow", "orange", "green"][thresholds.index(updated_radius_vals[gear_point])]
    self.get_logger().info(f"Picking up a {gear_color} gear with radius size of {updated_radius_vals[gear_point]}")
```
The gripper is then opened to pick up the gear and the robot moves directly above the gear. If the offset is needed, it is accounted for.
```python
self._call_open_gripper_service()  # opens the gripper
          
if offset_needed:
self._call_move_cartesian_smooth_service(
    move[0]+X_OFFSET, move[1]+Y_OFFSET, 0.0, 0.15, 0.2
)  # moves above the gear
else:
self._call_move_cartesian_smooth_service(
    move[0], move[1], 0.0, 0.15,0.2
)  # moves above the gear
```
After the initial move, a second check is done above the gear. This is done the same way as the detection before with one difference. If multiple gears are found, the correct gear is set to the closest gear to the center of the gripper.
```python
while (correct_gear in [[0.0,0.0,0.0],[None for _ in range(3)]] or sum(correct_gear)==0.0) and counter <3:
    counter+=1
    multiple_gears = MultipleGears(self.connected)
    rclpy.spin_once(multiple_gears)  # finds multiple gears if there are multiple
    self.connected = multiple_gears.connected
    while (
        sum([cent.count(None) for cent in multiple_gears.g_centers]) != 0
        or not multiple_gears.ran
    ):  # loops until it has run and until there are no None values
        multiple_gears.destroy_node()
        multiple_gears = MultipleGears(self.connected)
        rclpy.spin_once(multiple_gears)
        self.connected = multiple_gears.connected
    object_depth = ObjectDepth(multiple_gears.g_centers, multiple_gears.dist_points)
    rclpy.spin_once(object_depth)  # Gets the distance from the camera
    multiple_gears.destroy_node()
    object_depth.destroy_node()  # Destroys the node to avoid errors on next loop
    closest_gears =  object_depth.coordinates
    correct_gear_index = self.closest_to_center(closest_gears)
    correct_gear = closest_gears[self.closest_to_center(closest_gears)] if correct_gear_index>0 else [0 for _ in range(3)]
```
If the second check is not successful, the robot will still attempt to pick up the gear in the current position. If it is successful, the robot will move directly above it and pick it up.
```python
if correct_gear.count(0.0)>=1 or correct_gear.count(None)>=1:
    self.get_logger().error("Second check above gear did not work. Attempting to pick up with current position")
    self._call_pick_up_gear_coord_service(False,0.0,0.0, gear_point[2], object_width, False)
else:
self._call_pick_up_gear_coord_service(
    True, -1*correct_gear[1], -1*correct_gear[0],-1*correct_gear[2], object_width, False
)
last_point=(last_point[0]+-1*correct_gear[1] +X_OFFSET,last_point[1]+-1*correct_gear[0]+Y_OFFSET)
```
Finally, the robot will put the gear back down. To do this, the current joint positions are read using the call back, the put_down_force service is called, and the robot moves back to the joint position. The program then sets the offset_needed variable to false, as it has already been accounted for after the first gear.
```python
self._call_get_joint_positions()
self._call_put_down_force(0.1)
self._call_move_to_joint_position(self.current_joint_positions)
offset_needed = False
```
## _call_multiple_gears_single_scan
This function is very similar to `_call_pick_up_multiple_gears` except the starting position is up higher and the scan is only done at a single position. This higher position is moved to at the beginning of the funciton using this command:
```python
self._call_move_to_named_pose_service("high_scan")
```
Another major difference is that for the scan, the class that is used is not `MultipleGears`, but `MultipleGearsColor`. This class uses the color image from the camera to detect the gears instead of the depth image, which allows gears which are further away from the camera to be detected. Since there is only a single single position, the coordinates do not have to be saved in relation to any position other than the current positon.
```python
for coord in object_depth.coordinates:
    if coord.count(0.0)==0:  # adds coordinates if not all 0. Duplicates are removed later
        distances_from_home.append(
            (
                -1 * coord[1],
                -1 * coord[0],
                -1 * coord[2]
            )
        )
        updated_radius_vals[(
                -1 * coord[1],
                -1 * coord[0],
                -1 * coord[2])] = object_depth.radius_vals[coord]
```
Other than these small differences, everything else in the function is the same as `_call_pick_up_multiple_gears` except that the color scan is used instead of the depth scan.
## _call_multiple_gears_rotated_scan
This function is not working yet, but two positions are scanned at different angles and then the roobot records their positon in relation to the home position.
## _call_open_gripper_service
This funciton calls the service to open the gripper. There are no variables in the request.
## _call_pick_up_gear_coord_service
This function picks up a gear given coordinates. Also, using the `offset_bool` parameter, the offset can either be applied or not. Also, using the `default_up` parameter, the robot can move up the default amount or the amount set using the coordinates. This is the request for this service:
```python
z_movement = max(
    Z_TO_TABLE, z + Z_CAMERA_OFFSET
)  # z distance from the home position to where the gripper can grab the gear
self.get_logger().info(f"Picking up gear")
request = PickUpGear.Request()

request.x = x + (X_OFFSET if offset_bool else 0)
request.y = y + (Y_OFFSET if offset_bool else 0)

request.z = z_movement
request.object_width = object_width
request.default_up = default_up
```
## _call_put_gear_down_camera
This function uses the camera to put the gear down. A z value is still asked for as a fail safe so the robot does not move into an object. First, 4 variables are declared, the depth values which record the recorded z values from the camera, the x_center and y_center, which are used to declare the pixels for detection, and a counter so the robot does not scan infinitely if it can not detect the surface below.
```python
depth_vals = []
x_center = 100
y_center = 100
c=0
```
Then, inside of a loop the `ObjectDepth` class is used 