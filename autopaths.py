import math
import wpilib
import pathfinder as pf


# Robot attributes
WHEEL_DIAMETER = 0.5  # 6 inches
ENCODER_COUNTS_PER_REV = 360

# Pathfinder constants
MAX_VELOCITY = 5  # ft/s
MAX_ACCELERATION = 6


def get_trajectory(points, period=.02):
    # Set up the trajectory
    info, trajectory = pf.generate(points, pf.FIT_HERMITE_CUBIC, pf.SAMPLES_HIGH,
                                   dt=period,
                                   max_velocity=MAX_VELOCITY,
                                   max_acceleration=MAX_ACCELERATION,
                                   max_jerk=120.0)

    # Wheelbase Width = 2 ft
    modifier = pf.modifiers.TankModifier(trajectory).modify(2.0)
    left = modifier.getLeftTrajectory()
    right = modifier.getRightTrajectory()
    return left, modifier.source, right
