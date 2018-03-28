import pathfinder as pf


def get_trajectory(points, period=.02, max_velocity=5, max_acceleration=6, max_jerk=120, wheelbase_width=2):
    # Set up the trajectory
    info, trajectory = pf.generate(points, pf.FIT_HERMITE_CUBIC, pf.SAMPLES_HIGH,
                                   dt=period,
                                   max_velocity=max_velocity,
                                   max_acceleration=max_acceleration,
                                   max_jerk=max_jerk)

    modifier = pf.modifiers.TankModifier(trajectory).modify(wheelbase_width)
    left = modifier.getLeftTrajectory()
    right = modifier.getRightTrajectory()
    return left, modifier.source, right


def get_followers(left, encoder, encoder_counts_per_rev, wheel_diameter, max_velocity):

    Follower = pf.followers.EncoderFollower(left)
    Follower.configureEncoder(encoder.get(), encoder_counts_per_rev, wheel_diameter)
    Follower.configurePIDVA(1.0, 0.0, 0.0, 1 / max_velocity, 0)

    Follower = Follower

    return Follower



