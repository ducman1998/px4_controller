#!/usr/bin/env python3
"""Reusable PX4 ROS 2 offboard controller for local NED position waypoints."""

import math
from typing import List, Sequence, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from geometry_msgs.msg import Point, PoseStamped, TransformStamped
from nav_msgs.msg import Path
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint
from px4_msgs.msg import VehicleCommand, VehicleLocalPosition
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray


WaypointXY = Tuple[float, float]
PointNED = Tuple[float, float, float]


class OffboardPositionController(Node):
    """Base mission node. Subclasses provide relative local-NED x/y offsets."""

    def __init__(self, node_name: str, waypoints: Sequence[WaypointXY],
                 takeoff_altitude: float = 1.0, position_tolerance: float = 0.15,
                 waypoint_hold_duration: float = 3.0,
                 max_velocity: float = 0.25, vertical_speed: float = 0.20):
        super().__init__(node_name)
        self.waypoints = list(waypoints)
        self.takeoff_altitude = takeoff_altitude
        self.position_tolerance = position_tolerance
        self.waypoint_hold_duration = waypoint_hold_duration
        self.max_velocity = max_velocity
        self.vertical_speed = vertical_speed

        px4_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.offboard_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', px4_qos)
        self.setpoint_pub = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', px4_qos)
        self.command_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', px4_qos)
        self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position',
            self._local_position_callback, px4_qos)

        # Match the ROS 1 RViz display topics and frame names. PX4 local
        # coordinates are NED; ROS visualization convention is ENU.
        viz_qos = QoSProfile(depth=10)
        self.local_pose_pub = self.create_publisher(
            PoseStamped, '/waypoint_mission/local_pose', viz_qos)
        self.trajectory_pub = self.create_publisher(
            Path, '/waypoint_mission/trajectory', viz_qos)
        self.marker_pub = self.create_publisher(
            MarkerArray, '/waypoint_mission/markers', viz_qos)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.trajectory = Path()
        self.trajectory.header.frame_id = 'map'

        self.local_position = None
        self.origin: Point | None = None
        self.phase = 'wait_for_position'
        self.phase_ticks = 0
        self.waypoint_index = 0
        self.waypoint_hold_started = None
        self.commanded_position = None
        self.timer = self.create_timer(0.1, self._timer_callback)

        if not self.waypoints:
            raise ValueError('At least one waypoint is required')

        self.get_logger().info('Waiting for a valid PX4 local position estimate')

    def _local_position_callback(self, msg: VehicleLocalPosition) -> None:
        self.local_position = msg

    def _publish_visualization(self) -> None:
        if self.local_position is None or self.origin is None:
            return

        p = self.local_position
        stamp = self.get_clock().now().to_msg()
        position = Point(x=p.y, y=p.x, z=-p.z)

        pose = PoseStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = 'map'
        pose.pose.position = position
        # Convert PX4's NED heading to an ENU yaw for the RViz pose axes.
        enu_yaw = math.pi / 2.0 - p.heading
        pose.pose.orientation.z = math.sin(enu_yaw / 2.0)
        pose.pose.orientation.w = math.cos(enu_yaw / 2.0)
        self.local_pose_pub.publish(pose)

        self.trajectory.header.stamp = stamp
        self.trajectory.poses.append(pose)
        if len(self.trajectory.poses) > 2000:
            self.trajectory.poses.pop(0)
        self.trajectory_pub.publish(self.trajectory)

        view_tf = TransformStamped()
        view_tf.header.stamp = stamp
        view_tf.header.frame_id = 'map'
        view_tf.child_frame_id = 'waypoint_mission_center'
        offsets_x = [w[0] for w in self.waypoints]
        offsets_y = [w[1] for w in self.waypoints]
        # Transform the center of the route from NED into ENU.
        view_tf.transform.translation.x = self.origin[1] + (min(offsets_y) + max(offsets_y)) / 2.0
        view_tf.transform.translation.y = self.origin[0] + (min(offsets_x) + max(offsets_x)) / 2.0
        view_tf.transform.translation.z = position.z
        view_tf.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(view_tf)

        markers = MarkerArray()
        drone = Marker()
        drone.header.frame_id, drone.header.stamp = 'map', stamp
        drone.ns, drone.id = 'drone', 0
        drone.type, drone.action = Marker.CUBE, Marker.ADD
        drone.pose = pose.pose
        drone.scale.x, drone.scale.y, drone.scale.z = 0.45, 0.30, 0.10
        drone.color.r, drone.color.g, drone.color.b, drone.color.a = 0.15, 0.55, 1.0, 1.0
        markers.markers.append(drone)

        home = Point(x=self.origin[1], y=self.origin[0], z=-self.origin[2])
        axes = (((1, 0, 0), (1, 0, 0)), ((0, 1, 0), (0, 1, 0)), ((0, 0, 1), (0, 0, 1)))
        for axis_id, (direction, color) in enumerate(axes):
            axis = Marker()
            axis.header.frame_id, axis.header.stamp = 'map', stamp
            axis.ns, axis.id = 'home_frame', axis_id
            axis.type, axis.action = Marker.ARROW, Marker.ADD
            axis.points = [
                Point(x=home.x, y=home.y, z=home.z),
                Point(x=home.x + direction[0] * 0.45,
                      y=home.y + direction[1] * 0.45,
                      z=home.z + direction[2] * 0.45),
            ]
            axis.scale.x, axis.scale.y, axis.scale.z = 0.025, 0.05, 0.08
            axis.color.r, axis.color.g, axis.color.b, axis.color.a = (
                float(color[0]), float(color[1]), float(color[2]), 1.0)
            markers.markers.append(axis)

        takeoff_z = self.origin[2] - self.takeoff_altitude
        for waypoint_id, waypoint in enumerate(self.waypoints):
            marker = Marker()
            marker.header.frame_id, marker.header.stamp = 'map', stamp
            marker.ns, marker.id = 'waypoints', waypoint_id
            marker.type, marker.action = Marker.SPHERE, Marker.ADD
            marker.pose.position = Point(
                x=self.origin[1] + waypoint[1],
                y=self.origin[0] + waypoint[0],
                z=-takeoff_z,
            )
            marker.pose.orientation.w = 1.0
            marker.scale.x = marker.scale.y = marker.scale.z = 0.18
            if waypoint_id == self.waypoint_index and self.phase in ('waypoints', 'waypoint_hold'):
                marker.color.r, marker.color.g, marker.color.b = 1.0, 0.55, 0.0
            else:
                marker.color.r, marker.color.g, marker.color.b = 0.1, 1.0, 0.1
            marker.color.a = 1.0
            markers.markers.append(marker)

        status = Marker()
        status.header.frame_id, status.header.stamp = 'map', stamp
        status.ns, status.id = 'status', 0
        status.type, status.action = Marker.TEXT_VIEW_FACING, Marker.ADD
        status.pose.position = Point(x=position.x, y=position.y, z=position.z + 1.0)
        status.pose.orientation.w = 1.0
        status.scale.z = 0.22
        status.color.r = status.color.g = status.color.b = status.color.a = 1.0
        status.text = (
            f'Mode Offboard / {self.phase}\nAlt {position.z:.2f} m\n'
            f'Pos ENU {position.x:.2f}, {position.y:.2f}, {position.z:.2f} m\n'
            f'Vel ENU {p.vy:.2f}, {p.vx:.2f}, {-p.vz:.2f} m/s'
        )
        markers.markers.append(status)
        self.marker_pub.publish(markers)

    def _now_us(self) -> int:
        return self.get_clock().now().nanoseconds // 1000

    def _publish_offboard_heartbeat(self) -> None:
        msg = OffboardControlMode()
        msg.timestamp = self._now_us()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        self.offboard_pub.publish(msg)

    def _publish_position(self, target: PointNED) -> None:
        msg = TrajectorySetpoint()
        msg.timestamp = self._now_us()
        msg.position = [float(v) for v in target]
        msg.velocity = [math.nan, math.nan, math.nan]
        msg.acceleration = [math.nan, math.nan, math.nan]
        msg.jerk = [math.nan, math.nan, math.nan]
        msg.yaw = math.nan
        msg.yawspeed = math.nan
        self.setpoint_pub.publish(msg)

    def _slew_position(self, target: PointNED) -> PointNED:
        """Move the position target gradually to cap commanded flight speed."""
        if self.commanded_position is None:
            p = self.local_position
            self.commanded_position = (p.x, p.y, p.z)

        current = self.commanded_position
        dx, dy = target[0] - current[0], target[1] - current[1]
        horizontal_distance = math.hypot(dx, dy)
        horizontal_step = self.max_velocity * 0.1
        if horizontal_distance > horizontal_step:
            scale = horizontal_step / horizontal_distance
            x = current[0] + dx * scale
            y = current[1] + dy * scale
        else:
            x, y = target[0], target[1]

        z_error = target[2] - current[2]
        vertical_step = self.vertical_speed * 0.1
        z = current[2] + max(-vertical_step, min(vertical_step, z_error))
        if abs(z_error) <= vertical_step:
            z = target[2]

        self.commanded_position = (x, y, z)
        return self.commanded_position

    def _send_command(self, command: int, param1: float = 0.0,
                      param2: float = 0.0) -> None:
        msg = VehicleCommand()
        msg.timestamp = self._now_us()
        msg.param1 = float(param1)
        msg.param2 = float(param2)
        msg.command = command
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.command_pub.publish(msg)

    def _at_target(self, target: PointNED) -> bool:
        p = self.local_position
        dx, dy, dz = p.x - target[0], p.y - target[1], p.z - target[2]
        return math.sqrt(dx * dx + dy * dy + dz * dz) <= self.position_tolerance

    def _timer_callback(self) -> None:
        self._publish_offboard_heartbeat()

        if self.local_position is None or not (
                self.local_position.xy_valid and self.local_position.z_valid):
            return

        p = self.local_position
        if self.origin is None:
            self.origin = (p.x, p.y, p.z)
            self.commanded_position = self.origin
            self.get_logger().info(
                f'Local origin captured: NED ({p.x:.2f}, {p.y:.2f}, {p.z:.2f})')
        self._publish_visualization()

        takeoff_target = (self.origin[0], self.origin[1],
                          self.origin[2] - self.takeoff_altitude)

        if self.phase == 'wait_for_position':
            self._publish_position((p.x, p.y, p.z))
            self.phase_ticks += 1
            if self.phase_ticks >= 10:  # Stream setpoints for 1 second before Offboard.
                self._send_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
                self._send_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
                self.phase = 'takeoff'
                self.get_logger().info('Requested arm and Offboard mode; taking off')
            return

        if self.phase == 'takeoff':
            self._publish_position(self._slew_position(takeoff_target))
            if self._at_target(takeoff_target):
                self.phase = 'waypoints'
                self.get_logger().info('Takeoff altitude reached; starting waypoint route')
            return

        if self.phase in ('waypoints', 'waypoint_hold'):
            dx, dy = self.waypoints[self.waypoint_index]
            # Route waypoints control horizontal position only; hold the
            # altitude established by the takeoff setpoint.
            target = (self.origin[0] + dx, self.origin[1] + dy,
                      takeoff_target[2])
            commanded_target = self._slew_position(target)
            self._publish_position(commanded_target)
            if self.phase == 'waypoints' and self._at_target(target):
                self.phase = 'waypoint_hold'
                self.waypoint_hold_started = self.get_clock().now()
                self.get_logger().info(
                    f'Reached waypoint {self.waypoint_index + 1}/{len(self.waypoints)}; '
                    f'holding for {self.waypoint_hold_duration:.1f} s')
            elif self.phase == 'waypoint_hold':
                if not self._at_target(target):
                    self.waypoint_hold_started = self.get_clock().now()
                elif (self.get_clock().now() - self.waypoint_hold_started).nanoseconds >= int(
                        self.waypoint_hold_duration * 1e9):
                    self.waypoint_index += 1
                    self.waypoint_hold_started = None
                    if self.waypoint_index == len(self.waypoints):
                        self.phase = 'landing'
                        self.get_logger().info('Route complete; requesting PX4 auto land')
                        self._send_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
                    else:
                        self.phase = 'waypoints'
            return

        if self.phase == 'landing':
            # Keep streaming the return-to-launch position setpoint until PX4
            # changes modes and takes over the landing sequence.
            landing_hold = (self.origin[0], self.origin[1], takeoff_target[2])
            self._publish_position(self._slew_position(landing_hold))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = OffboardPositionController('offboard_position_controller', [(0.0, 0.0)])
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
