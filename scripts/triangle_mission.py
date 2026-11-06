#!/usr/bin/env python3
"""Fly a 1 m local-position triangle and request PX4 automatic landing."""

import math

import rclpy

from offboard_position_controller import OffboardPositionController


class TriangleMission(OffboardPositionController):
    def __init__(self, max_velocity: float = 0.25,
                 waypoint_hold_duration: float = 3.0):
        # Coordinates are (x, y) offsets in the PX4 local NED frame, relative
        # to the position captured at startup. The base controller holds the
        # takeoff altitude. Side length is 1 m; the last point closes the
        # triangle at the takeoff point.
        waypoints = [
            (0.0, 0.0),
            (0.75, -0.75),
            (0.75, 0.75),
            (-0.75, 0.75),
            (-0.75, -0.75),
            (0.0, 0.0),
        ]
        super().__init__(
            'triangle_mission', waypoints,
            takeoff_altitude=1.0,
            waypoint_hold_duration=waypoint_hold_duration,
            max_velocity=max_velocity,
        )


def main(args=None):
    rclpy.init(args=args)
    node = TriangleMission()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
