# HW2: Laser Occupancy-Grid Mapping
> first edition

by Jiaming Yang
## Data and method

The bag at `/hw2_data` contains LaserScan, IMU, and Odometry data. The basic
method uses the odometry pose as the vehicle trajectory. Its orientation has
already been produced by the encoder/IMU pipeline. Each valid laser endpoint
is transformed from the laser frame to the map frame and votes for one grid
cell. Cells with no endpoint evidence remain unknown.

The coordinate transform for a beam with range `r` and angle `a` is:

```text
p_laser = laser_offset + (r cos(a), r sin(a))
p_map = (x_robot, y_robot) + R(yaw_robot) p_laser
```

The map resolution is 0.10 m per cell. A sparse vote table is converted to a
dynamically sized `nav_msgs/OccupancyGrid` and published on `/map`.

## Run the simple voting map

```bash
cd /home/ubuntu/ros2_ws
colcon build --packages-select hw2_answer
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch hw2_answer hw2_answer.launch.py
```

## Run the Cartographer map

```bash
cd /home/ubuntu/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch hw2_answer hw2_cartographer.launch.py
```

## Comparison

Insert one screenshot from each launch here after running the bag.

| Aspect | Simple voting | Cartographer |
| --- | --- | --- |
| Pose source | Recorded odometry | Scan matching plus pose graph |
| Occupancy evidence | Laser endpoint counts | Probabilistic submaps |
| Drift correction | None | Local scan matching and loop closure |
| Expected result | Fast but sensitive to odometry drift | More continuous walls and better global consistency |

The final discussion should describe the observed wall continuity, noise,
alignment after revisiting an area, and any areas where Cartographer performs
better or worse than endpoint voting.
