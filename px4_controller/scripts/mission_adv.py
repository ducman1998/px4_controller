#!/usr/bin/env python
"""Take off with position setpoints, fly velocity waypoints, then land."""

import copy
import math
import rospy
import tf2_ros
from geometry_msgs.msg import Point, PoseStamped, TransformStamped, TwistStamped
from mavros_msgs.msg import PositionTarget, State
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker, MarkerArray


class AdvancedWaypointMission(object):
    """Use position setpoints for takeoff and velocity control for waypoints."""

    def __init__(
        self, waypoint_offsets=None, waypoint_hold_duration=2.0, takeoff_hold=2.0
    ):
        rospy.init_node("waypoint_mission_adv")
        self.state = State()
        self.pose = None
        self.velocity = None
        self.home = None
        self.home_position = None
        self.target = None
        self.waypoints = []
        self.phase = "prepare"
        self.last_request = rospy.Time(0)
        self.start_time = rospy.Time.now()
        self.setpoint_started = None
        self.warmup_setpoint_count = 0
        self.phase_started = None
        self.index = 0

        self.startup_delay = rospy.get_param("~startup_delay", 3.0)
        self.takeoff_altitude = rospy.get_param("~takeoff_altitude", 1.0)
        self.position_tolerance = rospy.get_param("~position_tolerance", 0.2)
        self.offboard_warmup = rospy.get_param("~offboard_warmup", 2.0)
        self.offboard_warmup_setpoints = rospy.get_param(
            "~offboard_warmup_setpoints", 40
        )
        self.takeoff_hold = rospy.get_param("~takeoff_hold", takeoff_hold)
        self.waypoint_hold_duration = rospy.get_param(
            "~waypoint_hold_duration", waypoint_hold_duration
        )
        self.waypoint_offsets = waypoint_offsets or [
            (0.0, 0.0), (1.0, 0.0), (0.5, 0.8660254), (0.0, 0.0),
        ]
        self.max_horizontal_speed = rospy.get_param("~max_horizontal_speed", 0.45)
        self.max_vertical_speed = rospy.get_param("~max_vertical_speed", 0.3)
        self.position_kp = rospy.get_param("~position_kp", 0.8)
        self.position_kp_z = rospy.get_param("~position_kp_z", 1.5)
        self.velocity_kd = rospy.get_param("~velocity_kd", 1.4)

        self.setpoint_pub = rospy.Publisher(
            "/mavros/setpoint_position/local", PoseStamped, queue_size=10
        )
        self.local_pose_pub = rospy.Publisher(
            "/waypoint_mission/local_pose", PoseStamped, queue_size=10
        )
        self.trajectory = Path()
        self.trajectory.header.frame_id = "map"
        self.trajectory_pub = rospy.Publisher(
            "/waypoint_mission/trajectory", Path, queue_size=10
        )
        self.marker_pub = rospy.Publisher(
            "/waypoint_mission/markers", MarkerArray, queue_size=10
        )
        self.view_tf_broadcaster = tf2_ros.TransformBroadcaster()
        self.velocity_pub = rospy.Publisher(
            "/mavros/setpoint_raw/local", PositionTarget, queue_size=10
        )
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
        self.mode_client = rospy.ServiceProxy("/mavros/set_mode", SetMode)
        self.arm_client = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)
        self.land_client = rospy.ServiceProxy("/mavros/cmd/land", CommandTOL)
        rospy.wait_for_service("/mavros/set_mode")
        rospy.wait_for_service("/mavros/cmd/arming")
        rospy.wait_for_service("/mavros/cmd/land")
        self.timer = rospy.Timer(rospy.Duration(0.05), self.tick)

    def pose_cb(self, msg):
        self.pose = msg
        if self.home is None:
            self.home = msg
            p = msg.pose.position
            self.home_position = (p.x, p.y, p.z)
            self.target = self.copy_pose(msg)
            self.waypoints = [
                (p.x + dx, p.y + dy, p.z + self.takeoff_altitude)
                for dx, dy in self.waypoint_offsets
            ]
            rospy.loginfo(
                "Captured local origin at (%.2f, %.2f, %.2f); takeoff target %.2f m above origin",
                msg.pose.position.x, msg.pose.position.y, msg.pose.position.z,
                self.takeoff_altitude,
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

    def state_cb(self, msg):
        previous_mode = self.state.mode
        previous_armed = self.state.armed
        self.state = msg
        if msg.mode != previous_mode or msg.armed != previous_armed:
            rospy.loginfo(
                "PX4 state changed: mode=%s armed=%s connected=%s",
                msg.mode or "unknown", msg.armed, msg.connected,
            )

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
        drone.color.r, drone.color.g, drone.color.b, drone.color.a = (
            0.15, 0.55, 1.0, 1.0
        )
        markers.markers.append(drone)

        if self.home_position is not None:
            hx, hy, hz = self.home_position
            axes = (
                ((1, 0, 0), (1, 0, 0)),
                ((0, 1, 0), (0, 1, 0)),
                ((0, 0, 1), (0, 0, 1)),
            )
            for axis_id, (direction, color) in enumerate(axes):
                axis = Marker()
                axis.header.frame_id, axis.header.stamp = frame_id, stamp
                axis.ns, axis.id = "home_frame", axis_id
                axis.type, axis.action = Marker.ARROW, Marker.ADD
                axis.points = [
                    Point(x=hx, y=hy, z=hz),
                    Point(
                        x=hx + direction[0] * 0.45,
                        y=hy + direction[1] * 0.45,
                        z=hz + direction[2] * 0.45,
                    ),
                ]
                axis.scale.x, axis.scale.y, axis.scale.z = 0.025, 0.05, 0.08
                axis.color.r, axis.color.g, axis.color.b, axis.color.a = (
                    color[0], color[1], color[2], 1.0
                )
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
        status.text = (
            "Mode %s\nAlt %.2f m\nPos %.2f, %.2f, %.2f m\n"
            "Vel ENU %.2f, %.2f, %.2f m/s"
        ) % (self.state.mode or "unknown", p.z, p.x, p.y, p.z, vx, vy, vz)
        markers.markers.append(status)
        self.marker_pub.publish(markers)

    @staticmethod
    def copy_pose(source):
        target = PoseStamped()
        target.header.frame_id = source.header.frame_id or "map"
        target.pose = copy.deepcopy(source.pose)
        return target

    def set_target(self, x, y, z):
        target = PoseStamped()
        target.header.frame_id = self.pose.header.frame_id or "map"
        target.pose.orientation = copy.deepcopy(self.pose.pose.orientation)
        target.pose.position.x = x
        target.pose.position.y = y
        target.pose.position.z = z
        self.target = target

    def publish_target(self, now):
        self.target.header.stamp = now
        self.setpoint_pub.publish(self.target)

    def request_mode(self, mode, now):
        if (now - self.last_request).to_sec() < 1.0:
            return
        try:
            response = self.mode_client(0, mode)
            self.last_request = now
            rospy.loginfo("Requested PX4 mode %s (mode_sent=%s)", mode, response.mode_sent)
        except rospy.ServiceException as exc:
            rospy.logwarn("Request for PX4 mode %s failed: %s", mode, exc)

    def request_arm(self, now):
        if (now - self.last_request).to_sec() < 1.0:
            return
        try:
            response = self.arm_client(True)
            self.last_request = now
            rospy.loginfo("Arming request accepted=%s", response.success)
        except rospy.ServiceException as exc:
            rospy.logwarn("Arming request failed: %s", exc)

    def distance_to_target(self):
        p = self.pose.pose.position
        t = self.target.pose.position
        dx, dy, dz = t.x - p.x, t.y - p.y, t.z - p.z
        return math.sqrt(dx * dx + dy * dy + dz * dz)

    def publish_local_velocity(self, vx, vy, vz, now):
        msg = PositionTarget()
        msg.header.stamp = now
        msg.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        msg.type_mask = (
            PositionTarget.IGNORE_PX | PositionTarget.IGNORE_PY |
            PositionTarget.IGNORE_PZ | PositionTarget.IGNORE_AFX |
            PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ |
            PositionTarget.IGNORE_YAW | PositionTarget.IGNORE_YAW_RATE
        )
        # MAVROS converts ROS ENU velocity components to PX4's NED frame.
        msg.velocity.x, msg.velocity.y, msg.velocity.z = vx, vy, vz
        self.velocity_pub.publish(msg)

    def velocity_to_waypoint(self, target):
        p = self.pose.pose.position
        dx, dy, dz = target[0] - p.x, target[1] - p.y, target[2] - p.z
        if self.velocity is None:
            evx = evy = evz = 0.0
        else:
            v = self.velocity.twist.linear
            evx, evy, evz = v.x, v.y, v.z

        # PD position-to-velocity control: proportional position error drives
        # toward the waypoint; measured velocity provides derivative damping.
        vx = self.position_kp * dx - self.velocity_kd * evx
        vy = self.position_kp * dy - self.velocity_kd * evy
        vz = self.position_kp_z * dz - self.velocity_kd * evz

        # Limit horizontal speed by its vector magnitude so diagonal commands
        # do not exceed the same speed limit as straight-line commands.
        horizontal_speed = math.sqrt(vx * vx + vy * vy)
        if horizontal_speed > self.max_horizontal_speed:
            scale = self.max_horizontal_speed / horizontal_speed
            vx *= scale
            vy *= scale
        vz = max(-self.max_vertical_speed, min(self.max_vertical_speed, vz))
        return vx, vy, vz, math.sqrt(dx * dx + dy * dy + dz * dz)

    def tick(self, _event):
        if not self.state.connected or self.pose is None or self.target is None:
            return
        now = rospy.Time.now()
        if (now - self.start_time).to_sec() < self.startup_delay:
            return

        if self.phase == "prepare":
            if self.setpoint_started is None:
                self.setpoint_started = now
            # Stream zero-velocity targets before asking PX4 to enter OFFBOARD.
            self.publish_local_velocity(0.0, 0.0, 0.0, now)
            self.warmup_setpoint_count += 1
            if (
                self.warmup_setpoint_count < self.offboard_warmup_setpoints or
                (now - self.setpoint_started).to_sec() < self.offboard_warmup
            ):
                return
            self.phase = "offboard"

        if self.phase == "offboard":
            if self.state.mode != "OFFBOARD":
                self.publish_local_velocity(0.0, 0.0, 0.0, now)
                self.request_mode("OFFBOARD", now)
                return
            # After PX4 accepts OFFBOARD, hold current position while arming.
            p = self.pose.pose.position
            self.set_target(p.x, p.y, p.z)
            self.publish_target(now)
            if not self.state.armed:
                self.request_arm(now)
                return
            home = self.home.pose.position
            self.set_target(home.x, home.y, home.z + self.takeoff_altitude)
            self.phase = "takeoff"
            rospy.loginfo("OFFBOARD active and armed; climbing to position setpoint")

        if self.phase == "takeoff":
            self.publish_target(now)
            if self.state.mode != "OFFBOARD":
                self.request_mode("OFFBOARD", now)
                return
            if self.distance_to_target() <= self.position_tolerance:
                if self.phase_started is None:
                    self.phase_started = now
                    rospy.loginfo("Takeoff position reached; holding briefly")
                elif (now - self.phase_started).to_sec() >= self.takeoff_hold:
                    p = self.target.pose.position
                    self.waypoints = [
                        (self.home.pose.position.x + dx,
                         self.home.pose.position.y + dy, p.z)
                        for dx, dy in self.waypoint_offsets
                    ]
                    self.index = 0
                    self.set_target(*self.waypoints[0])
                    self.phase = "route"

        elif self.phase == "route":
            target = self.waypoints[self.index]
            vx, vy, vz, distance = self.velocity_to_waypoint(target)
            if distance <= self.position_tolerance:
                self.publish_local_velocity(0.0, 0.0, 0.0, now)
            else:
                self.publish_local_velocity(vx, vy, vz, now)
            if self.state.mode != "OFFBOARD":
                self.request_mode("OFFBOARD", now)
                return
            if distance <= self.position_tolerance:
                if self.phase_started is None:
                    self.phase_started = now
                elif (now - self.phase_started).to_sec() >= self.waypoint_hold_duration:
                    self.phase_started = None
                    self.index += 1
                    if self.index >= len(self.waypoints):
                        self.phase = "land"
                        rospy.loginfo("Route complete; sending MAVROS land command")
                    else:
                        self.set_target(*self.waypoints[self.index])
                        rospy.loginfo("Proceeding to waypoint %d/%d", self.index + 1, len(self.waypoints))
            else:
                self.phase_started = None

        if self.phase == "land":
            self.publish_local_velocity(0.0, 0.0, 0.0, now)
        if self.phase == "land" and (now - self.last_request).to_sec() >= 1.0:
            try:
                response = self.land_client(0.0, 0.0, 0.0, 0.0, 0.0)
                self.last_request = now
                rospy.loginfo("Land command accepted=%s", response.success)
                if response.success:
                    self.phase = "landing"
            except rospy.ServiceException as exc:
                self.last_request = now
                rospy.logwarn("Land command failed: %s", exc)


if __name__ == "__main__":
    try:
        AdvancedWaypointMission()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
