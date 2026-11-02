#!/usr/bin/env python
"""Fly a 1 m equilateral triangle, return to launch, and land."""

import os
import sys

import rospy

# catkin's generated executable wrapper runs this source file via exec(), so
# its scripts directory may not be present on Python's import path.
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from mission import WaypointMission


if __name__ == "__main__":
    try:
        triangle_offsets = [
            (0.0, 0.0),
            (1.0, 0.0),
            (0.5, 0.8660254),
            (0.0, 0.0),
        ]
        WaypointMission(
            waypoint_offsets=triangle_offsets,
            waypoint_hold_duration=3.0,
            hover_duration=5.0,
        )
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
