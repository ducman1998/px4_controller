# PX4 Autopilot Modes for PX4 Vision Dev Kit v1.5 (PX4 v1.13.3)

This guide covers multicopter modes on our PX4 Vision Dev Kit v1.5 (Pixhawk 6C),
running PX4 firmware **v1.13.3**. Do not change firmware versions: another build
may not support the installed range and optical-flow sensors. Mode behavior and
parameters below refer to v1.13.3 and this vehicle's configuration.

## Modes at a glance

| Mode | Control input or behavior | Position and frame behavior | Notes |
| --- | --- | --- | --- |
| **Position** | Manual roll/pitch commands horizontal movement; yaw commands yaw rate; throttle commands vertical speed. By default, horizontal stick deflection commands acceleration. | The pilot's horizontal directions are relative to the vehicle heading. PX4 uses its local position estimate to brake and hold position when the sticks are centered. | `MPC_POS_MODE` selects how stick input maps to motion. In current PX4, value `0` is Direct velocity; the default value `4` is Acceleration based. Verify the enum for the installed firmware. |
| **Altitude** | Manual roll/pitch control tilt; yaw commands yaw rate; throttle commands climb or descent. | Horizontal movement is relative to vehicle heading. PX4 holds altitude when centered, but does not brake or hold horizontal position. Wind and momentum can carry the vehicle. | Useful when a valid horizontal position estimate is unavailable, but it requires a local altitude estimate. |
| **Stabilized** | Roll/pitch command attitude, yaw commands yaw rate, and throttle controls thrust (with behavior affected by `MPC_THR_CURVE`). | No position hold and no altitude hold. Centered roll/pitch levels the vehicle; it can drift horizontally and altitude must be managed by the pilot. | Requires pilot input. It does not compensate for wind to hold position. |
| **Hold** | PX4 stops and hovers without continuous pilot input. | Holds local position and altitude against disturbances. If engaged below `NAV_MIN_LTR_ALT`, PX4 first climbs to that minimum loiter altitude. | Requires an adequate 3D position estimate. Stick movement normally switches a multicopter to Position mode. |
| **Mission** | PX4 executes a mission plan uploaded to the flight controller. | Mission navigation uses the estimator and the mission's coordinate references; typical outdoor missions use global position. | A TAKEOFF mission item climbs to at least `MIS_TAKEOFF_ALT` before continuing. If the vehicle is already airborne when the mission starts, TAKEOFF is treated as a waypoint. |
| **Return (RTL)** | PX4 automatically returns to a safe destination and lands. | Uses a global position and home/rally reference. Default multicopter behavior climbs as needed to a safe return altitude, flies home or to a rally point, then descends and lands. | Behavior depends on RTL parameters and configured return type. Review `RTL_RETURN_ALT`, `RTL_DESCEND_ALT`, and `RTL_LAND_DELAY`. |
| **Takeoff** | PX4 takes off vertically, then waits for further input. | Requires a valid local position estimate. It holds position after reaching its target altitude. | Target altitude is set by `MIS_TAKEOFF_ALT`; climb speed by `MPC_TKO_SPEED`. RC stick movement normally changes to Position mode. |
| **Land** | PX4 descends and lands at the position where Land was engaged, then disarms after landing by default. | Requires a valid local position estimate; it does not require GPS/global position. | Descent rate is set by `MPC_LAND_SPEED`. RC stick movement normally changes to Position mode. |
| **Descend** | PX4 automatically descends while maintaining attitude. | Open-loop horizontally: it does not hold horizontal position, so wind can cause drift. | A failsafe fallback, not a pilot-selectable mode. It can be entered when position-dependent modes cannot continue; the exact failsafe path depends on configuration. |
| **Offboard** | An external controller streams setpoints to PX4. | Setpoint dimensions and coordinate frame depend on the interface and message. | Allows custom trajectory tracking. PX4 requires a suitable state estimate and a continuous stream above 2 Hz, started before entering Offboard. If the stream is lost, PX4 applies the configured Offboard-loss action. |

### Offboard options in this project

1. **ROS1 with MAVROS:** The current controller publishes
   `mavros_msgs/PositionTarget`. In this project it commands local position or
   velocity setpoints; it does not command attitude, body rates, or thrust.

2. **ROS2 with PX4 messages:** `px4_msgs::msg::TrajectorySetpoint` supports
   NED position, velocity, acceleration, yaw, and yaw-speed setpoints (jerk is
   for logging). Attitude, body-rate, and thrust control require their own PX4
   setpoint messages; they are not fields in `TrajectorySetpoint`.

The direct ROS2 PX4 interface may require firmware/interface support beyond the
current setup. Keep the aircraft on PX4 v1.13.3 unless a replacement is
confirmed to support its range and optical-flow sensors.

## Coordinate-frame notes

- **Vehicle-relative pilot input:** Roll/pitch stick directions are interpreted
  relative to the vehicle's heading (forward/back and left/right). PX4 rotates
  these commands into its navigation frame.
- **Local position hold and navigation:** PX4 uses its local estimator frame to
  track position. The estimator may obtain local position from GPS or from a
  valid vision/motion-capture source; a mode's exact estimator requirements
  still apply.
- **MAVROS Offboard setpoints:** MAVROS ROS topics commonly use ROS ENU
  conventions, while PX4/MAVLink uses NED conventions. MAVROS performs frame
  conversion for the raw local setpoint topic. Do not manually swap axes or
  negate vertical velocity without checking the exact message and MAVROS
  plugin behavior.

## Parameters mentioned here

| Parameter | Purpose |
| --- | --- |
| `MPC_POS_MODE` | Maps Position-mode stick input to motion. Current PX4 uses `0` for Direct velocity and `4` for Acceleration based; check the installed release because enum values can differ. |
| `MIS_TAKEOFF_ALT` | Takeoff altitude used by Takeoff mode and mission takeoff behavior. |
| `MPC_TKO_SPEED` | Vertical ascent speed during takeoff. |
| `NAV_MIN_LTR_ALT` | Minimum altitude Hold mode climbs to when engaged below it. |
| `RTL_RETURN_ALT` | Safe return altitude used by the default Return behavior. |
| `RTL_DESCEND_ALT` | Altitude reached at the return destination before the final landing sequence. |
| `RTL_LAND_DELAY` | Delay before landing after reaching the return destination. |
| `MPC_LAND_SPEED` | Descent speed during Land mode. |
| `COM_OF_LOSS_T` | Time before PX4 treats a lost Offboard stream as an Offboard-loss event. |
| `COM_OBL_RC_ACT` | Action selected for Offboard loss when RC is available. |

Parameter names, values, and behavior are firmware-version dependent. Read the
parameter metadata from the firmware actually installed on this aircraft.

## References

- [PX4 Vision Autonomy Development Kit](https://docs.px4.io/v1.13/en/complete_vehicles/px4_vision_kit) — kit documentation.
- [PX4 v1.13 flight modes overview](https://docs.px4.io/v1.13/en/getting_started/flight_modes)
- [Position mode](https://docs.px4.io/v1.13/en/flight_modes/position_mc)
- [Altitude mode](https://docs.px4.io/v1.13/en/flight_modes/altitude_mc)
- [Manual/Stabilized mode](https://docs.px4.io/v1.13/en/flight_modes/manual_stabilized_mc)
- [Hold mode](https://docs.px4.io/v1.13/en/flight_modes/hold)
- [Mission mode](https://docs.px4.io/v1.13/en/flight_modes/mission)
- [Return mode](https://docs.px4.io/v1.13/en/flight_modes/return)
- [Takeoff mode](https://docs.px4.io/v1.13/en/flight_modes/takeoff)
- [Land mode](https://docs.px4.io/v1.13/en/flight_modes/land)
- [Safety configuration and failsafes](https://docs.px4.io/v1.13/en/config/safety) — includes automatic fallback behavior.
- [Offboard mode](https://docs.px4.io/v1.13/en/flight_modes/offboard)
- [`TrajectorySetpoint.msg`](https://github.com/PX4/PX4-Autopilot/blob/main/msg/versioned/TrajectorySetpoint.msg)
- [Multicopter setpoint tuning](https://docs.px4.io/v1.13/en/config_mc/mc_trajectory_tuning)
