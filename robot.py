import magicbot
import wpilib
import ctre
import wpilib.drive
import math
from robotpy_ext.common_drivers import navx

class MyRobot(magicbot.MagicRobot):

    WHEEL_DIAMETER = .5
    ENCODER_COUNTS_PER_REV = 1000

    def createObjects(self):
        self.init_drive_train()


    def init_drive_train(self):
        fl, bl, fr, br = (30, 40, 50, 10)  # practice bot
        #br, fr, bl, fl = (1, 7, 2, 5)  # on competition robot

        self.br_motor = ctre.wpi_talonsrx.WPI_TalonSRX(br)
        self.bl_motor = ctre.wpi_talonsrx.WPI_TalonSRX(bl)
        self.fl_motor = ctre.wpi_talonsrx.WPI_TalonSRX(fl)
        self.fr_motor = ctre.wpi_talonsrx.WPI_TalonSRX(fr)
        self.fr_motor.setInverted(True)
        self.br_motor.setInverted(True)

        self.gyro = navx.AHRS.create_spi()

        self.joystick = wpilib.Joystick(0)
        self.joystick2 = wpilib.Joystick(1)

        self.robot_drive = wpilib.RobotDrive(self.fl_motor, self.bl_motor, self.fr_motor, self.br_motor)


        self.r_encoder = wpilib.Encoder(2, 3)
        self.r_encoder.setDistancePerPulse((math.pi * self.WHEEL_DIAMETER) / self.ENCODER_COUNTS_PER_REV)


    def teleopInit(self):
        self.fl_motor.setQuadraturePosition(0,0)
        self.br_motor.setQuadraturePosition(0,0)

    def teleopPeriodic(self):
        # wheel diameter =  6 inches = .1524 meters
        self.ticks_per_rev = 1000
        evr = "{:10.4f}".format(self.get_right_distance())
        evl = "{:10.4f}".format(self.get_left_distance())
        vy = "{:10.4f}".format(self.gyro.getVelocityY())
        ay = "{:10.4f}".format(self.gyro.getWorldLinearAccelY())
        vx = "{:10.4f}".format(self.gyro.getVelocityX())
        ax = "{:10.4f}".format(self.gyro.getWorldLinearAccelX())


        print("enc right: ", evr," enc left: ", evl, " vy: ", vy, " ay: ", ay, " vx: ", vx, "ax: ", ax)
        self.robot_drive.arcadeDrive(self.joystick.getX(), self.joystick.getY())


    def get_left_distance(self):
        return self.fl_motor.getQuadraturePosition() * (math.pi * self.WHEEL_DIAMETER) / self.ENCODER_COUNTS_PER_REV

    def get_right_distance(self):
        return -self.br_motor.getQuadraturePosition() * (math.pi * self.WHEEL_DIAMETER) / self.ENCODER_COUNTS_PER_REV



if __name__ == '__main__':
    wpilib.run(MyRobot)