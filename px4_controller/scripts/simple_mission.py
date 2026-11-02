#!/usr/bin/env python
"""Take off, hover briefly, and land using MAVROS in ROS 1."""

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode


class SimpleMission(object):
    """Run a short PX4 takeoff, hover, and land sequence."""

    def __init__(self):
        rospy.init_node("simple_mission")
        self.state = State()
        self.start_time = rospy.Time.now()
        self.last_request = rospy.Time(0)
        self.phase = "wait"
        self.hover_started = None
        self.takeoff_started = None
        self.stability_started = None
        self.stability_min_z = None
        self.stability_max_z = None
        self.local_z = None

        self.startup_delay = rospy.get_param("~startup_delay", 3.0)
        self.hover_duration = rospy.get_param("~hover_duration", 5.0)
        self.takeoff_settle_delay = rospy.get_param("~takeoff_settle_delay", 5.0)
        self.altitude_stability_duration = rospy.get_param(
            "~altitude_stability_duration", 2.0
        )
        self.altitude_stability_tolerance = rospy.get_param(
            "~altitude_stability_tolerance", 0.15
        )

        self.state_sub = rospy.Subscriber(
            "/mavros/state", State, self.state_cb, queue_size=10
        )
        self.pose_sub = rospy.Subscriber(
            "/mavros/local_position/pose", PoseStamped, self.pose_cb, queue_size=10
        )
        rospy.loginfo("Waiting %.1f seconds before starting simple mission", self.startup_delay)

        self.mode_client = rospy.ServiceProxy("/mavros/set_mode", SetMode)
        self.arm_client = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)
        self.timer = rospy.Timer(rospy.Duration(0.1), self.tick)

    def state_cb(self, msg):
        self.state = msg

    def pose_cb(self, msg):
        self.local_z = msg.pose.position.z

    def request_arm(self, now):
        if (now - self.last_request).to_sec() < 1.5:
            return
        try:
            response = self.arm_client(True)
            self.last_request = now
            rospy.loginfo("Arming requested (accepted=%s)", response.success)
        except rospy.ServiceException as exc:
            rospy.logwarn("Arming request failed: %s", exc)

    def request_mode(self, mode, now):
        if (now - self.last_request).to_sec() < 1.5:
            return
        try:
            response = self.mode_client(0, mode)
            self.last_request = now
            rospy.loginfo("Requested mode %s (accepted=%s)", mode, response.mode_sent)
        except rospy.ServiceException as exc:
            rospy.logwarn("Request for mode %s failed: %s", mode, exc)

    def tick(self, _event):
        if not self.state.connected:
            rospy.logwarn_throttle(5.0, "Waiting for MAVROS connection")
            return

        now = rospy.Time.now()
        if self.phase == "wait":
            if (now - self.start_time).to_sec() < self.startup_delay:
                return
            self.phase = "arm"
            rospy.loginfo("Startup delay complete; preparing takeoff")

        if self.phase == "arm":
            if not self.state.armed:
                self.request_arm(now)
                return
            self.phase = "takeoff_mode"

        if self.phase == "takeoff_mode":
            if self.state.mode != "AUTO.TAKEOFF":
                self.request_mode("AUTO.TAKEOFF", now)
                return
            self.phase = "takeoff"
            self.takeoff_started = now
            rospy.loginfo(
                "PX4 takeoff mode active; waiting %.1f seconds before checking "
                "altitude stability",
                self.takeoff_settle_delay,
            )

        if self.phase == "takeoff":
            if self.state.mode not in ("AUTO.TAKEOFF", "AUTO.LOITER"):
                self.request_mode("AUTO.TAKEOFF", now)
                return
            if self.local_z is None:
                rospy.logwarn_throttle(
                    5.0,
                    "Waiting for /mavros/local_position/pose before confirming takeoff",
                )
            elif (now - self.takeoff_started).to_sec() >= self.takeoff_settle_delay:
                if self.stability_started is None:
                    self.stability_started = now
                    self.stability_min_z = self.local_z
                    self.stability_max_z = self.local_z
                else:
                    self.stability_min_z = min(self.stability_min_z, self.local_z)
                    self.stability_max_z = max(self.stability_max_z, self.local_z)
                    altitude_range = self.stability_max_z - self.stability_min_z
                    stable_for = (now - self.stability_started).to_sec()
                    if altitude_range > self.altitude_stability_tolerance:
                        self.stability_started = now
                        self.stability_min_z = self.local_z
                        self.stability_max_z = self.local_z
                    elif stable_for >= self.altitude_stability_duration:
                        self.hover_started = now
                        self.phase = "hover"
                        rospy.loginfo(
                            "Takeoff altitude stable at z=%.2f m; hovering for %.1f seconds",
                            self.local_z,
                            self.hover_duration,
                        )

        if self.phase == "hover":
            if self.state.mode not in ("AUTO.LOITER", "AUTO.TAKEOFF"):
                rospy.logwarn_throttle(5.0, "Waiting in hover phase; current PX4 mode is %s", self.state.mode)
                return
            if (now - self.hover_started).to_sec() >= self.hover_duration:
                self.phase = "land"
                rospy.loginfo("Hover complete; requesting AUTO.LAND")

        if self.phase == "land":
            if self.state.mode != "AUTO.LAND":
                self.request_mode("AUTO.LAND", now)
            else:
                rospy.loginfo("PX4 AUTO.LAND mode active; simple mission complete")
                self.phase = "done"
        elif self.phase == "done":
            return


if __name__ == "__main__":
    try:
        SimpleMission()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
