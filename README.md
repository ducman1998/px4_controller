# PX4 ROS 2 Offboard Controller

ROS 2 Python offboard examples for PX4 using `px4_msgs` and the Micro XRCE-DDS
client/agent bridge. This guide targets PX4 **v1.14.4** and ROS 2 **Humble** on
Ubuntu 22.04. PX4 v1.14 uses uXRCE-DDS; it does not use the PX4 v1.13
Fast-RTPS bridge.

The PX4 client runs in PX4 firmware and exchanges DDS data with
`MicroXRCEAgent` on the companion computer. ROS 2 nodes publish and subscribe
to the exposed PX4 topics. The ROS 2 message definitions must match the PX4
firmware, so this workspace uses the `px4_msgs` **`release/1.14`** branch.

For ROS 2 setup and protocol details, see the [PX4 v1.14 ROS 2 User Guide](https://docs.px4.io/v1.14/en/ros/ros2_comm),
[uXRCE-DDS guide](https://docs.px4.io/v1.14/en/middleware/uxrce_dds), and
[Offboard mode guide](https://docs.px4.io/v1.14/en/flight_modes/offboard).

## Contents

- `scripts/offboard_position_controller.py` — reusable controller that streams
  `OffboardControlMode` and `TrajectorySetpoint` messages in PX4 local NED.
- `scripts/triangle_mission.py` — example waypoint mission built on the
  controller. Its `max_velocity` and `waypoint_hold_duration` constructor
  arguments set the horizontal setpoint speed limit and pause at each point.
- `rviz/waypoint_mission_ros2.rviz` — RViz 2 view for the live pose, path,
  waypoint markers, and status markers.

The controller folder currently has no ROS 2 `package.xml` or launch file.
Build `px4_msgs` with colcon, then run the Python scripts directly as shown
below.

## Requirements

- Ubuntu 22.04 and ROS 2 Humble (`ros-humble-desktop` provides RViz 2).
- PX4-Autopilot v1.14.4 for SITL or the intended flight controller firmware.
- `px4_msgs` from the `release/1.14` branch.
- Micro XRCE-DDS Agent on the computer that connects to PX4's uXRCE-DDS client.
- QGroundControl is optional for monitoring PX4.

PX4 v1.14.4 firmware support for the PX4 Vision Dev Kit sensors and board
configuration must be confirmed for the specific airframe before updating a
real flight controller. A matching ROS message branch alone does not verify
hardware compatibility.

## Build the ROS 2 workspace

The workspace is `ros2_ws` at the repository root. Check that `px4_msgs` is
using the v1.14 definitions:

```bash
cd /path/to/repository/ros2_ws/src/px4_msgs
git checkout release/1.14
```

Install missing dependencies and build the workspace:

```bash
cd /path/to/repository/ros2_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
```

Source both ROS 2 and the workspace in every terminal that runs a node:

```bash
cd /path/to/repository/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
```

## Micro XRCE-DDS Agent

The DDS domain ID is currently **10**. PX4's `UXRCE_DDS_DOM_ID` parameter and
the `ROS_DOMAIN_ID` environment variable on the companion computer and ROS 2
operator computer must all be set to `10`. For example, source ROS 2 in each
ROS terminal with:

```bash
export ROS_DOMAIN_ID=10
```

### (1) Simulation (SITL)

PX4 SITL connects to the agent over UDP port `8888`. Install the agent using
the PX4 v1.14 documented Snap channel:

```bash
sudo snap install micro-xrce-dds-agent --edge
```

Start the UDP agent in its own terminal before starting SITL:

```bash
micro-xrce-dds-agent udp4 -p 8888
```

After PX4 SITL and the agent are both running, check the client status in the
PX4 shell:

```text
uxrce_dds_client status
```

If it is not connected, verify the agent is listening on UDP port `8888` and
that SITL and ROS 2 use `ROS_DOMAIN_ID=10`.

### (2) Real PX4 Vision 1.5

**The companion computer is already configured to start the agent as a system
service** over the USB serial link. The service runs:

```bash
/usr/bin/snap run micro-xrce-dds-agent serial --dev /dev/ttyUSB0 -b 921600
```

In the PX4 shell, check the client status:

```text
uxrce_dds_client status
```

Leave that service in control of the serial agent. Do not manually start a
second agent on `/dev/ttyUSB0`. PX4's uXRCE-DDS client must be configured for
the matching flight-controller serial port and baud rate. PX4 v1.14's client
is included in firmware; see the [PX4 v1.14 uXRCE-DDS guide](https://docs.px4.io/v1.14/en/middleware/uxrce_dds)
for client setup and hardware-specific port configuration.

If the client is not connected, check these PX4 settings and the companion
service:

- Set `UXRCE_DDS_CFG=TELEM2` so the client uses TELEM2 to communicate with the
  companion computer's agent.
- Disable MAVLink on TELEM2 with `MAV_1_CONFIG=Disabled (0)` so the DDS client
  can use the serial port. The PX4 v1.14 parameter is named `MAV_1_CONFIG`.
- Make sure `UXRCE_DDS_DOM_ID` on PX4 and `ROS_DOMAIN_ID` on the UP Core and
  operator computer are all `10`.
- Confirm the companion service uses `/dev/ttyUSB0` at `921600` baud and that
  the cable is connected to PX4's TELEM2 port.

## Run in SITL

Use separate terminals for the UDP agent, PX4, mission, and RViz.

1. Start the agent as described above.

2. In the PX4-Autopilot v1.14.4 checkout, start SITL. Gazebo Classic is shown
   here:

   ```bash
   cd /path/to/PX4-Autopilot
   git checkout v1.14.4
   git submodule update --init --recursive
   ROS_DOMAIN_ID=10 make px4_sitl gazebo-classic
   ```

   PX4 v1.14 SITL starts the uXRCE-DDS client, which should connect to the
   agent at `127.0.0.1:8888`. Check the PX4 and agent terminals for connection
   messages.

   With both PX4 simulation and the agent running, open a ROS 2 terminal with
   `ROS_DOMAIN_ID=10` and list the drone topics:

   ```bash
   source /opt/ros/humble/setup.bash
   export ROS_DOMAIN_ID=10
   ros2 topic list
   ```

   The list should include PX4 topics such as `/fmu/out/vehicle_local_position`
   and `/fmu/in/trajectory_setpoint`.

3. In a ROS 2 terminal, source the workspace and run the mission:

   ```bash
   cd /path/to/repository/ros2_ws
   source /opt/ros/humble/setup.bash
   export ROS_DOMAIN_ID=10
   source install/setup.bash
   python3 src/px4_controller/scripts/triangle_mission.py
   ```

4. In another sourced ROS 2 terminal, start RViz 2 with the included view:

   ```bash
   cd /path/to/repository/ros2_ws
   source /opt/ros/humble/setup.bash
   export ROS_DOMAIN_ID=10
   source install/setup.bash
   rviz2 -d src/px4_controller/rviz/waypoint_mission_ros2.rviz
   ```

The node expects PX4 topics including `/fmu/out/vehicle_local_position` and
publishes to `/fmu/in/offboard_control_mode`, `/fmu/in/trajectory_setpoint`,
and `/fmu/in/vehicle_command`. Check that the PX4 v1.14.4
`src/modules/uxrce_dds_client/dds_topics.yaml` exposes these topics for the
firmware build. `px4_msgs` must use matching v1.14 definitions.

## Run with a real vehicle

The companion computer already runs the Micro XRCE-DDS Agent as a startup
service over `/dev/ttyUSB0` at `921600` baud. Confirm that the service is active
and that the PX4 uXRCE-DDS client on the flight controller is configured for
the matching serial connection. Do not start the SITL UDP agent on the real
drone, or launch a second serial agent. The companion computer must also have
ROS 2 Humble available to the mission node. The SITL UDP localhost settings do
not apply to the aircraft.

Once the client and agent report a connection, inspect the ROS 2 graph before
starting the mission:

```bash
ros2 topic list | grep '^/fmu/'
ros2 topic echo /fmu/out/vehicle_local_position
```

Then run the mission and RViz 2 using the commands in the SITL section. The
controller arms, requests Offboard, takes off with position setpoints, follows
its local-position waypoints, then sends PX4 `VEHICLE_CMD_NAV_LAND` through
`/fmu/in/vehicle_command` to request automatic landing.

Offboard requires a valid position estimate and a continuous
`OffboardControlMode` stream. PX4 requires the stream to be active for at least
one second before entering Offboard and exits Offboard if the stream is lost.
Check the vehicle state in QGroundControl, keep a working manual recovery
method available, and begin with a controlled test area and low-risk setup.

## Tune the example mission

`TriangleMission` accepts its horizontal setpoint speed cap in meters per
second and the time to pause at each waypoint in seconds:

```python
node = TriangleMission(max_velocity=0.15, waypoint_hold_duration=4.0)
```

The defaults are `max_velocity=0.25` m/s and `waypoint_hold_duration=3.0` s.
The controller also limits the vertical setpoint slew separately in
`OffboardPositionController`.
