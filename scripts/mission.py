#!/usr/bin/env python
"""Fly the waypoint route using MAVROS and ROS 1."""

import math

import rospy
import tf2_ros
from geometry_msgs.msg import Point, PoseStamped, TransformStamped, TwistStamped
from mavros_msgs.msg import PositionTarget, State
from mavros_msgs.srv import CommandBool, SetMode
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker, MarkerArray


class WaypointMission(object):
    """Use PX4 takeoff/land modes and body velocities for the route."""

    def __init__(self, waypoint_offsets=None, waypoint_hold_duration=0.0):
        rospy.init_node("waypoint_mission")
        self.state = State()
        self.pose = None
        self.velocity = None
        self.home_position = None
        self.waypoints = []
        self.waypoint_offsets = waypoint_offsets or [
            (0.0, 0.0), (5.0, 0.0), (5.0, 1.25), (0.0, 1.25),
            (0.0, 2.5), (5.0, 2.5), (5.0, 3.75), (0.0, 3.75),
            (0.0, 5.0), (5.0, 5.0),
        ]
        self.index = 0
        self.phase = "takeoff_mode"
        self.last_request = rospy.Time.now()
        self.last_progress_log = rospy.Time.now()
        self.mission_start_time = rospy.Time.now()
        self.startup_delay = 10.0
        self.setpoint_stream_started = None
        self.takeoff_altitude_reached_at = None

        self.takeoff_altitude = 1.0
        self.takeoff_altitude_hold_duration = 2.0
        self.offboard_preparation_duration = 3.0
        self.position_tolerance = 0.20
        self.waypoint_hold_duration = rospy.get_param(
            "~waypoint_hold_duration", waypoint_hold_duration
        )
        self.waypoint_hold_started = None
        self.max_horizontal_speed = 0.6
        self.max_vertical_speed = 0.4
        self.velocity_gain = 0.8

        self.trajectory = Path()
        self.trajectory.header.frame_id = "map"
        self.velocity_pub = rospy.Publisher(
            "/mavros/setpoint_raw/local", PositionTarget, queue_size=10
        )
        self.local_pose_pub = rospy.Publisher(
            "/waypoint_mission/local_pose", PoseStamped, queue_size=10
        )
        self.trajectory_pub = rospy.Publisher(
            "/waypoint_mission/trajectory", Path, queue_size=10
        )
        self.marker_pub = rospy.Publisher(
            "/waypoint_mission/markers", MarkerArray, queue_size=10
        )
        self.view_tf_broadcaster = tf2_ros.TransformBroadcaster()
        self.state_sub = rospy.Subscriber(
            "/mavros/state", State, self.state_cb, queue_size=10
        )
        self.pose_sub = rospy.Subscriber(
            "/mavros/local_position/pose", PoseStamped, self.pose_cb, queue_size=10
        )
        self.velocity_sub = rospy.Subscriber(
            "/mavros/local_position/velocity_local", TwistStamped,
            self.velocity_cb, queue_size=10
        )
        rospy.wait_for_service("/mavros/set_mode")
        rospy.wait_for_service("/mavros/cmd/arming")
        self.mode_client = rospy.ServiceProxy("/mavros/set_mode", SetMode)
        self.arm_client = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)
        self.timer = rospy.Timer(rospy.Duration(0.05), self.tick)
        rospy.loginfo("Mission will start in 10 seconds")

    def state_cb(self, msg):
        self.state = msg

    def pose_cb(self, msg):
        self.pose = msg
        if self.home_position is None:
            p = msg.pose.position
            self.home_position = (p.x, p.y, p.z)
            x, y, z = self.home_position
            altitude = z + self.takeoff_altitude
            self.waypoints = [
                (x + dx, y + dy, altitude)
                for dx, dy in self.waypoint_offsets
            ]
            route_x = [offset[0] for offset in self.waypoint_offsets]
            route_y = [offset[1] for offset in self.waypoint_offsets]
            rospy.loginfo(
                "Home captured at (%.2f, %.2f, %.2f); takeoff target z=%.2f m; "
                "%d waypoints span %.2f x %.2f m",
                x, y, z, altitude, len(self.waypoints),
                max(route_x) - min(route_x), max(route_y) - min(route_y)
            )

        self.publish_view_frame(msg)
        self.local_pose_pub.publish(msg)
        self.trajectory.header.stamp = msg.header.stamp
        self.trajectory.header.frame_id = msg.header.frame_id or "map"
        self.trajectory.poses.append(msg)
        if len(self.trajectory.poses) > 2000:
            del self.trajectory.poses[0]
        self.trajectory_pub.publish(self.trajectory)
        self.publish_visual_markers(msg)

    def velocity_cb(self, msg):
        self.velocity = msg

    def publish_view_frame(self, pose):
        view_frame = TransformStamped()
        view_frame.header.stamp = rospy.Time.now()
        view_frame.header.frame_id = pose.header.frame_id or "map"
        view_frame.child_frame_id = "waypoint_mission_center"
        offset_x = [offset[0] for offset in self.waypoint_offsets]
        offset_y = [offset[1] for offset in self.waypoint_offsets]
        view_frame.transform.translation.x = self.home_position[0] + (
            min(offset_x) + max(offset_x)
        ) / 2.0
        view_frame.transform.translation.y = self.home_position[1] + (
            min(offset_y) + max(offset_y)
        ) / 2.0
        view_frame.transform.translation.z = pose.pose.position.z
        view_frame.transform.rotation.w = 1.0
        self.view_tf_broadcaster.sendTransform(view_frame)

    def publish_visual_markers(self, pose):
        frame_id = pose.header.frame_id or "map"
        stamp = rospy.Time.now()
        markers = MarkerArray()
        drone = Marker()
        drone.header.frame_id, drone.header.stamp = frame_id, stamp
        drone.ns, drone.id = "drone", 0
        drone.type, drone.action = Marker.CUBE, Marker.ADD
        drone.pose = pose.pose
        drone.scale.x, drone.scale.y, drone.scale.z = 0.45, 0.30, 0.10
        drone.color.r, drone.color.g, drone.color.b, drone.color.a = 0.15, 0.55, 1.0, 1.0
        markers.markers.append(drone)

        if self.home_position is not None:
            hx, hy, hz = self.home_position
            axes = (((1, 0, 0), (1, 0, 0)), ((0, 1, 0), (0, 1, 0)), ((0, 0, 1), (0, 0, 1)))
            for axis_id, (direction, color) in enumerate(axes):
                axis = Marker()
                axis.header.frame_id, axis.header.stamp = frame_id, stamp
                axis.ns, axis.id = "home_frame", axis_id
                axis.type, axis.action = Marker.ARROW, Marker.ADD
                axis.points = [Point(x=hx, y=hy, z=hz), Point(
                    x=hx + direction[0] * 0.45, y=hy + direction[1] * 0.45,
                    z=hz + direction[2] * 0.45)]
                axis.scale.x, axis.scale.y, axis.scale.z = 0.025, 0.05, 0.08
                axis.color.r, axis.color.g, axis.color.b, axis.color.a = color[0], color[1], color[2], 1.0
                markers.markers.append(axis)

        for waypoint_id, waypoint in enumerate(self.waypoints):
            marker = Marker()
            marker.header.frame_id, marker.header.stamp = frame_id, stamp
            marker.ns, marker.id = "waypoints", waypoint_id
            marker.type, marker.action = Marker.SPHERE, Marker.ADD
            marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = waypoint
            marker.pose.orientation.w = 1.0
            marker.scale.x = marker.scale.y = marker.scale.z = 0.18
            if waypoint_id == self.index and self.phase == "route":
                marker.color.r, marker.color.g, marker.color.b = 1.0, 0.55, 0.0
            else:
                marker.color.r, marker.color.g, marker.color.b = 0.1, 1.0, 0.1
            marker.color.a = 1.0
            markers.markers.append(marker)

        p = pose.pose.position
        if self.velocity is None:
            vx = vy = vz = 0.0
        else:
            linear = self.velocity.twist.linear
            vx, vy, vz = linear.x, linear.y, linear.z
        status = Marker()
        status.header.frame_id, status.header.stamp = frame_id, stamp
        status.ns, status.id = "status", 0
        status.type, status.action = Marker.TEXT_VIEW_FACING, Marker.ADD
        status.pose.position.x, status.pose.position.y = p.x, p.y
        status.pose.position.z = p.z + 1.0
        status.pose.orientation.w = 1.0
        status.scale.z = 0.22
        status.color.r = status.color.g = status.color.b = status.color.a = 1.0
        status.text = "Mode %s\nAlt %.2f m\nPos %.2f, %.2f, %.2f m\nVel ENU %.2f, %.2f, %.2f m/s" % (
            self.state.mode or "unknown", p.z, p.x, p.y, p.z, vx, vy, vz)
        markers.markers.append(status)
        self.marker_pub.publish(markers)

    def publish_body_velocity(self, vx, vy, vz):
        msg = PositionTarget()
        msg.header.stamp = rospy.Time.now()
        msg.coordinate_frame = PositionTarget.FRAME_BODY_NED
        msg.type_mask = (PositionTarget.IGNORE_PX | PositionTarget.IGNORE_PY |
                         PositionTarget.IGNORE_PZ | PositionTarget.IGNORE_AFX |
                         PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ |
                         PositionTarget.IGNORE_YAW | PositionTarget.IGNORE_YAW_RATE)
        msg.velocity.x, msg.velocity.y, msg.velocity.z = vx, vy, vz
        self.velocity_pub.publish(msg)

    def request_mode(self, mode, now):
        if (now - self.last_request).to_sec() >= 1.5:
            try:
                self.mode_client(0, mode)
                self.last_request = now
            except rospy.ServiceException as exc:
                rospy.logwarn("SetMode failed: %s", exc)

    def request_arm(self, now):
        if (now - self.last_request).to_sec() >= 1.5:
            try:
                self.arm_client(True)
                self.last_request = now
            except rospy.ServiceException as exc:
                rospy.logwarn("Arming request failed: %s", exc)

    def body_velocity_to_target(self, target):
        p = self.pose.pose.position
        dx, dy, dz = target[0] - p.x, target[1] - p.y, target[2] - p.z
        q = self.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y*q.y + q.z*q.z))
        forward = math.cos(yaw) * dx + math.sin(yaw) * dy
        left = -math.sin(yaw) * dx + math.cos(yaw) * dy
        clamp = lambda v, limit: max(-limit, min(limit, v))
        vx = clamp(self.velocity_gain * forward, self.max_horizontal_speed)
        vy = clamp(self.velocity_gain * left, self.max_horizontal_speed)
        vz = clamp(self.velocity_gain * dz, self.max_vertical_speed)
        return vx, vy, vz, math.sqrt(dx*dx + dy*dy + dz*dz)

    def tick(self, _event):
        if self.pose is None or not self.state.connected or not self.waypoints:
            return
        now = rospy.Time.now()
        if (now - self.mission_start_time).to_sec() < self.startup_delay:
            return
        position = self.pose.pose.position
        takeoff_target_z = self.waypoints[0][2]

        if self.phase == "takeoff_mode":
            if not self.state.armed:
                self.request_arm(now)
                return
            if self.state.mode != "AUTO.TAKEOFF":
                self.request_mode("AUTO.TAKEOFF", now)
                return
            self.phase = "takeoff"
            rospy.loginfo("AUTO.TAKEOFF active; waiting for climb")

        if self.phase == "takeoff":
            if self.state.mode not in ("AUTO.TAKEOFF", "AUTO.LOITER"):
                if (now - self.last_progress_log).to_sec() >= 2.0:
                    rospy.logwarn("Takeoff handoff waiting: mode=%s, z=%.2f m, target=%.2f m.", self.state.mode, position.z, takeoff_target_z)
                    self.last_progress_log = now
                return
            if position.z < takeoff_target_z - self.position_tolerance:
                self.takeoff_altitude_reached_at = None
                if (now - self.last_progress_log).to_sec() >= 2.0:
                    rospy.loginfo("Taking off: z=%.2f m, target=%.2f m", position.z, takeoff_target_z)
                    self.last_progress_log = now
                return
            if self.takeoff_altitude_reached_at is None:
                self.takeoff_altitude_reached_at = now
                rospy.loginfo("Takeoff altitude reached; confirming before Offboard")
                return
            if (now - self.takeoff_altitude_reached_at).to_sec() < self.takeoff_altitude_hold_duration:
                return
            self.waypoints = [(x, y, position.z) for x, y, _ in self.waypoints]
            self.phase = "offboard_prepare"
            self.setpoint_stream_started = now
            rospy.loginfo("Takeoff altitude reached; streaming velocity setpoints before requesting OFFBOARD")

        if self.phase == "offboard_prepare":
            if self.state.mode not in ("AUTO.TAKEOFF", "AUTO.LOITER", "OFFBOARD"):
                if (now - self.last_progress_log).to_sec() >= 2.0:
                    rospy.logwarn("Offboard handoff stopped: PX4 is in %s. Not overriding this mode.", self.state.mode)
                    self.last_progress_log = now
                return
            self.publish_body_velocity(0.0, 0.0, 0.0)
            if (now - self.setpoint_stream_started).to_sec() < self.offboard_preparation_duration:
                return
            if self.state.mode != "OFFBOARD":
                self.request_mode("OFFBOARD", now)
                return
            self.phase = "route"
            self.index = 0
            self.last_progress_log = now
            rospy.loginfo("OFFBOARD active; starting body-velocity route")

        if self.phase == "route":
            if self.waypoint_hold_started is not None:
                self.publish_body_velocity(0.0, 0.0, 0.0)
                if (now - self.waypoint_hold_started).to_sec() < self.waypoint_hold_duration:
                    return
                self.waypoint_hold_started = None
                self.index += 1
                self.last_progress_log = now
                if self.index >= len(self.waypoints):
                    self.phase = "landing"
                    self.request_mode("AUTO.LAND", now)
                    rospy.loginfo("Route complete; AUTO.LAND requested")
                    return

            target = self.waypoints[self.index]
            vx, vy, vz, distance = self.body_velocity_to_target(target)
            self.publish_body_velocity(vx, vy, vz)
            if distance < self.position_tolerance:
                self.waypoint_hold_started = now
                self.publish_body_velocity(0.0, 0.0, 0.0)
                self.last_progress_log = now
                rospy.loginfo(
                    "Reached waypoint %d; holding for %.1f seconds",
                    self.index + 1, self.waypoint_hold_duration
                )
                return
            if (now - self.last_progress_log).to_sec() >= 2.0:
                rospy.loginfo("Waypoint %d: distance=%.2f m, body velocity=(%.2f, %.2f, %.2f) m/s", self.index + 1, distance, vx, vy, vz)
                self.last_progress_log = now
        elif self.phase == "landing" and self.state.mode != "AUTO.LAND":
            self.request_mode("AUTO.LAND", now)


if __name__ == "__main__":
    try:
        WaypointMission()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
