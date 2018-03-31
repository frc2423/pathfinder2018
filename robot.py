import magicbot
import wpilib
import ctre
import wpilib.drive
import math
from robotpy_ext.common_drivers import navx
from wpilib.timer import Timer

class MyRobot(magicbot.MagicRobot):

    WHEEL_DIAMETER = .5
    ENCODER_COUNTS_PER_L_REV = 1440
    ENCODER_COUNTS_PER_R_REV = 881
    previous_time = None
    previous_distance = None
    dt = 0
    dd = 0

    def createObjects(self):
        self.init_drive_train()
        self.timer = Timer()

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

    def teleopInit(self):
        self.fl_motor.setQuadraturePosition(0,0)
        self.br_motor.setQuadraturePosition(0,0)
        self.timer.start()

    def teleopPeriodic(self):
        if self.timer.get() > 1:
            self.calculate_deltas()
            self.timer.reset()

        # wheel diameter =  6 inches = .1524 meters
        self.ticks_per_rev = 1000
        evr = "{:10.4f}".format(self.get_right_distance())
        evl = "{:10.4f}".format(self.get_left_distance())
        vy = "{:10.4f}".format(self.gyro.getVelocityY())
        ay = "{:10.4f}".format(self.gyro.getWorldLinearAccelY())
        vx = "{:10.4f}".format(self.gyro.getVelocityX())
        ax = "{:10.4f}".format(self.gyro.getWorldLinearAccelX())

       # print("enc right: ", evr," enc left: ", evl, " vy: ", vy, " ay: ", ay, " vx: ", vx, "ax: ", ax)
        print('speed:', self.get_speed())

        #left = "{:10.4f}".format(self.fl_motor.getQuadraturePosition())
        #right = "{:10.4f}".format(self.br_motor.getQuadraturePosition())

        #print('left:', left, 'right:', right)

        self.robot_drive.arcadeDrive(self.joystick.getX(), self.joystick.getY())

    def get_left_distance(self):
        circumference = self.WHEEL_DIAMETER * math.pi
        encoder_distance = self.fl_motor.getQuadraturePosition()
        distance = (encoder_distance / self.ENCODER_COUNTS_PER_L_REV) * circumference
        return distance

    def get_right_distance(self):
        circumference = self.WHEEL_DIAMETER * math.pi
        encoder_distance = self.br_motor.getQuadraturePosition()
        distance = (encoder_distance / self.ENCODER_COUNTS_PER_R_REV) * circumference
        return distance


    def calculate_deltas(self):
        current_time = Timer.getFPGATimestamp()
        self.dt = 0 if self.previous_time is None else current_time - self.previous_time

        current_distance = self.get_right_distance()
        self.dd = 0 if self.previous_distance is None else current_distance - self.previous_distance

        self.previous_time = current_time
        self.previous_distance = current_distance

    def get_speed(self):
        velocity = 0 if self.dt == 0 else self.dd/self.dt
        return velocity

if __name__ == '__main__':
    wpilib.run(MyRobot)