#!/usr/bin/env python3
#
# This example demonstrates using robotpy-pathfinder in the pyfrc simulator
# with a working physics modules such that you can see the path being traced
#
# Note that the pyfrc physics aren't particularly realistic (in particular,
# friction and momentum are ignored), so performance of these exact parameters
# on a real robot won't be so great.
#

import math
import wpilib
import ctre
from robotpy_ext.common_drivers import navx
import pathfinder as pf
import pickle
import os.path

# max speed is 12ft/s

class MyRobot(wpilib.TimedRobot):
    '''Main robot class'''
    
    # Robot attributes
    WHEEL_DIAMETER = 0.5 # 6 inches
    ENCODER_COUNTS_PER_L_REV = 1440
    ENCODER_COUNTS_PER_R_REV = 881
    
    # Pathfinder constants
    MAX_VELOCITY = 5 # ft/s
    MAX_ACCELERATION = 5
    
    def robotInit(self):
        '''Robot-wide initialization code should go here'''
        
        self.lstick = wpilib.Joystick(0)
        self.rstick = wpilib.Joystick(1)

        fl, bl, fr, br = (30, 40, 50, 10)  # practice bot
        # br, fr, bl, fl = (1, 7, 2, 5)  # on competition robot

        self.br_motor = ctre.wpi_talonsrx.WPI_TalonSRX(br)
        self.bl_motor = ctre.wpi_talonsrx.WPI_TalonSRX(bl)
        self.fl_motor = ctre.wpi_talonsrx.WPI_TalonSRX(fl)
        self.fr_motor = ctre.wpi_talonsrx.WPI_TalonSRX(fr)
        self.fr_motor.setInverted(True)
        self.br_motor.setInverted(True)
        
        # Position gets automatically updated as robot moves
        self.gyro = navx.AHRS.create_spi()

        self.robot_drive = wpilib.RobotDrive(self.fl_motor, self.bl_motor, self.fr_motor, self.br_motor)


    
    def autonomousInit(self):

        self.fl_motor.setQuadraturePosition(0, 0)
        self.br_motor.setQuadraturePosition(0, 0)
        trajectory = self.get_trajectory()

        # Wheelbase Width = 2 ft
        modifier = pf.modifiers.TankModifier(trajectory).modify(2.25)

        # Do something with the new Trajectories...
        left = modifier.getLeftTrajectory()
        right = modifier.getRightTrajectory()
        
        leftFollower = pf.followers.EncoderFollower(left)
        leftFollower.configureEncoder(0, self.ENCODER_COUNTS_PER_L_REV, self.WHEEL_DIAMETER)
        leftFollower.configurePIDVA(0.5, 0.0, 0.0, 1 / self.MAX_VELOCITY, 0)
        
        rightFollower = pf.followers.EncoderFollower(right)
        rightFollower.configureEncoder(0, self.ENCODER_COUNTS_PER_R_REV, self.WHEEL_DIAMETER)
        rightFollower.configurePIDVA(0.5, 0.0, 0.0, 1 / self.MAX_VELOCITY, 0)
        
        self.leftFollower = leftFollower
        self.rightFollower = rightFollower
        
        # This code renders the followed path on the field in simulation (requires pyfrc 2018.2.0+)
        if wpilib.RobotBase.isSimulation():
            from pyfrc.sim import get_user_renderer
            renderer = get_user_renderer()
            if renderer:
                renderer.draw_pathfinder_trajectory(left, color='#0000ff', offset=(-1,0))
                renderer.draw_pathfinder_trajectory(modifier.source, color='#00ff00', show_dt=1.0, dt_offset=0.0)
                renderer.draw_pathfinder_trajectory(right, color='#0000ff', offset=(1,0))

    def get_trajectory(self):

        # Set up the trajectory
        points = [
            pf.Waypoint(0, 0, 0),
            pf.Waypoint(30, 3, 0),
        ]

        pickle_file = os.path.join(os.path.dirname(__file__), 'trajectory.pickle')

        if wpilib.RobotBase.isSimulation():
            # generate the trajectory here
            info, trajectory = pf.generate(points, pf.FIT_HERMITE_CUBIC, pf.SAMPLES_HIGH,
                                           dt=self.getPeriod(),
                                           max_velocity=self.MAX_VELOCITY,
                                           max_acceleration=self.MAX_ACCELERATION,
                                           max_jerk=100.0)

            # and then write it out
            with open(pickle_file, 'wb') as fp:
                pickle.dump(trajectory, fp)
        else:
            with open(pickle_file, 'rb') as fp:
                trajectory = pickle.load(fp)

        return trajectory


    def autonomousPeriodic(self):
        
        l = self.leftFollower.calculate(self.fl_motor.getQuadraturePosition())
        r = self.rightFollower.calculate(-self.br_motor.getQuadraturePosition())


        print('encoder_left:', self.fl_motor.getQuadraturePosition())
        print('encoder_right:', -self.br_motor.getQuadraturePosition())
        print('l:', -l)
        print('r:', r)

        gyro_heading = -self.gyro.getAngle()    # Assuming the gyro is giving a value in degrees
        desired_heading = pf.r2d(self.leftFollower.getHeading())   # Should also be in degrees

        # This is a poor man's P controller
        angleDifference = pf.boundHalfDegrees(desired_heading - gyro_heading)
        turn = 5 * (-1.0/80.0) * angleDifference
        
        #l = l + turn
        #r = r - turn

        # -1 is forward, so invert both values
        self.robot_drive.tankDrive(l, -r)
    
    def teleopPeriodic(self):
        #self.robot_drive.tankDrive(.6, .6)
        self.robot_drive.tankDrive(-self.lstick.getY(), self.rstick.getY())
        #self.robot_drive.arcadeDrive(self.lstick.getX(), self.lstick.getY())


    def get_left_distance(self):
        return self.fl_motor.getQuadraturePosition() * (math.pi * self.WHEEL_DIAMETER) / self.ENCODER_COUNTS_PER_REV

    def get_right_distance(self):
        return -self.br_motor.getQuadraturePosition() * (math.pi * self.WHEEL_DIAMETER) / self.ENCODER_COUNTS_PER_REV


if __name__ == '__main__':
    wpilib.run(MyRobot)
