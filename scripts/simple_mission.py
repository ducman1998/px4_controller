#!/usr/bin/env python
"""Take off, hover briefly, and land using MAVROS in ROS 1."""

import rospy
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

        self.startup_delay = rospy.get_param("~startup_delay", 3.0)
        self.hover_duration = rospy.get_param("~hover_duration", 10.0)

        self.state_sub = rospy.Subscriber(
            "/mavros/state", State, self.state_cb, queue_size=10
        )
        rospy.loginfo("Waiting %.1f seconds before starting simple mission", self.startup_delay)

        self.mode_client = rospy.ServiceProxy("/mavros/set_mode", SetMode)
        self.arm_client = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)
        self.timer = rospy.Timer(rospy.Duration(0.1), self.tick)

    def state_cb(self, msg):
        self.state = msg

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
            rospy.loginfo("PX4 takeoff mode active; waiting for takeoff and loiter")

        if self.phase == "takeoff":
            # PX4 reports AUTO.LOITER after its takeoff climb has completed.
            if self.state.mode == "AUTO.LOITER":
                self.hover_started = now
                self.phase = "hover"
                rospy.loginfo("Takeoff complete; hovering for %.1f seconds", self.hover_duration)
            elif self.state.mode != "AUTO.TAKEOFF":
                self.request_mode("AUTO.TAKEOFF", now)
                return

        if self.phase == "hover":
            if self.state.mode not in ("AUTO.LOITER", "AUTO.TAKEOFF"):
                rospy.logwarn_throttle(5.0, "Waiting in hover phase; current PX4 mode is %s", self.state.mode)
                return
            if self.state.mode == "AUTO.TAKEOFF":
                # Restart the hover clock if PX4 is still climbing.
                self.hover_started = None
                self.phase = "takeoff"
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
