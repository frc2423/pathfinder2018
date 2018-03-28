import magicbot
import wpilib
import ctre
import wpilib.drive
import math
from robotpy_ext.common_drivers import navx

class MyRobot(magicbot.MagicRobot):

    WHEEL_DIAMETER = .5
    ENCODER_COUNTS_PER_L_REV = 1440
    ENCODER_COUNTS_PER_R_REV = 881

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


        #print("enc right: ", evr," enc left: ", evl, " vy: ", vy, " ay: ", ay, " vx: ", vx, "ax: ", ax)

        left = "{:10.4f}".format(self.fl_motor.getQuadraturePosition())
        right = "{:10.4f}".format(self.br_motor.getQuadraturePosition())


        print('left:', left, 'right:', right)

        self.robot_drive.arcadeDrive(self.joystick.getX(), self.joystick.getY())


    def get_left_distance(self):

        cir= .5 * math.pi
        left = self.fl_motor.getQuadraturePosition()
        left_distens= (left / self.ENCODER_COUNTS_PER_L_REV) * cir
        return left_distens
    def get_right_distance(self):
        cir = .5 * math.pi
        right=self.br_motor.setQuadraturePosition()
        right_distens= (right / self.ENCODER_COUNTS_PER_R_REV) * cir
        return right_distens
if __name__ == '__main__':
    wpilib.run(MyRobot)