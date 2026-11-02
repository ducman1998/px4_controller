## PX4 Vision ROS 1 Workspace

ROS 1 Noetic waypoint controller and MAVROS bridge for the PX4 Vision Dev Kit
v1.5 (Pixhawk 6C). This drone uses PX4 firmware **v1.13.3**. Do not change the
firmware: another build may not support its range and optical-flow sensors.

See [PX4-AUTOPILOT-MODES.md](docs/PX4-AUTOPILOT-MODES.md) for the flight-mode
summary and the control behavior used by this drone.

## Installation

### Common requirements

- Ubuntu 20.04 and ROS 1 Noetic. `ros-base` is sufficient; install `desktop`
  if you need RViz.
- `catkin_tools` and `rosdep` build tools.
- GeographicLib tools and datasets for MAVROS.

Install only packages that are missing on the computer. For a minimal ROS
install, install ROS Noetic base; install RViz only if you want the visualizer.
MAVROS source is included in this workspace, so build it here instead of
installing a second MAVROS copy from apt.

If ROS Noetic is missing:

```
sudo apt install ros-noetic-ros-base
```

If you want RViz and it is missing:

```
sudo apt install ros-noetic-rviz
```

If catkin, rosdep, or GeographicLib tools are missing:

```
sudo apt update
sudo apt install ros-noetic-catkin python3-catkin-tools python3-rosdep geographiclib-tools libgeographic-dev
```

Initialize `rosdep` once per machine. If it is already initialized, omit its
first command in the block below. Then install missing workspace dependencies
and build the workspace. Replace `/path/to/repository` with the actual checkout
path. Repeat dependency installation and the build on each computer that will
run ROS packages from this workspace.

```
sudo rosdep init
rosdep update
source /opt/ros/noetic/setup.bash
cd /path/to/repository/ros1_ws
rosdep install --from-paths src --ignore-src -r -y
catkin build
```

Install GeographicLib data if it is not already present:

```
cd /path/to/repository/ros1_ws
sudo src/mavros/mavros/scripts/install_geographiclib_datasets.sh
```

### Simulation requirements

- PX4-Autopilot source at the `v1.13.3` release ref, with submodules initialized.
  Complete the PX4 setup by following the
  [PX4 v1.13 installation guide](https://docs.px4.io/v1.13/en/dev_setup/dev_env_linux_ubuntu).
- Gazebo Classic SITL. Use the PX4-Autopilot installation instructions for
  simulator dependencies.
- QGroundControl is optional for monitoring the simulated vehicle.

### Real-drone requirements

- PX4 Vision Dev Kit v1.5 running its current PX4 v1.13.3 firmware.
- QGroundControl on the operator computer.
- ROS Noetic and MAVROS on the drone's companion computer. Configure the MAVROS
  FCU URL for the Pixhawk serial device and baud rate used on the drone.
- Network connectivity between the companion computer and operator computer.
- Connect to the drone's `mixhawk4` Wi-Fi network (password: `mixhawk4`) before
  running with the real drone.

## Mission launch files

Run a mission only after MAVROS is connected and ROS topics are available.

### Configured waypoint route

`waypoint_mission.launch` runs `mission.py` and follows the configured waypoint
route. RViz starts by default. Set `use_rviz:=false` to disable it.

```
roslaunch px4_controller waypoint_mission.launch
```

### Triangle demo

`waypoint_mission_demo.launch` runs a short triangle route, returns to the
launch point, and holds briefly at each waypoint. RViz starts by default.

```
roslaunch px4_controller waypoint_mission_demo.launch
```

### Takeoff and landing check

`simple_mission.py` takes off, hovers briefly, and lands without following
waypoints.

```
rosrun px4_controller simple_mission.py
```

## Run in simulation

Use separate terminals for PX4, MAVROS, and the mission.

1. From the PX4-Autopilot repository, select the v1.13.3 release and start SITL
   with Gazebo Classic:

```
cd /path/to/PX4-Autopilot
git checkout v1.13.3
git submodule update --init --recursive
make px4_sitl gazebo
```

2. Start QGroundControl if you want to monitor the simulated drone.

3. Start MAVROS with the SITL UDP connection:

```
source /opt/ros/noetic/setup.bash
source /path/to/repository/ros1_ws/devel/setup.bash
roslaunch mavros px4.launch fcu_url:="udp://:14540@127.0.0.1:14557"
```

4. Start the waypoint mission in another terminal:

```
source /opt/ros/noetic/setup.bash
source /path/to/repository/ros1_ws/devel/setup.bash
roslaunch px4_controller waypoint_mission.launch use_rviz:=false
```

   Remove `use_rviz:=false` or set it to `true` to open the included RViz view.

## Run on the real drone

The ROS master and MAVROS run on the drone's companion computer. Its default
IP address is `192.168.4.2`. The operator computer runs the mission node and
connects to the companion's ROS master. Both machines must be reachable over
the ROS network. Replace the operator IP example if it differs on your network.

1. Connect the companion computer and operator computer to the `mixhawk4`
   Wi-Fi network using password `mixhawk4`.

2. On the companion computer, start the MAVROS bridge. `roslaunch` starts the
   local ROS master if one is not already running. Use the Pixhawk serial
   port and baud rate.

```
source /opt/ros/noetic/setup.bash
source /path/to/repository/ros1_ws/devel/setup.bash
roslaunch mavros px4.launch fcu_url:=/dev/ttyUSB0:921600
```

3. Open QGroundControl on the operator computer and connect over the configured
   telemetry link.

4. On the operator computer, start the mission. Set `OPERATOR_IP` to the
   computer's address on the network shared with the drone.

```
export ROS_MASTER_URI="http://192.168.4.2:11311"
source /opt/ros/noetic/setup.bash
source /path/to/repository/ros1_ws/devel/setup.bash
roslaunch px4_controller waypoint_mission.launch use_rviz:=true
```

The waypoint mission arms, takes off, enters Offboard, follows the configured
route, and requests landing. Confirm the vehicle and mode in QGroundControl
before starting the mission.



