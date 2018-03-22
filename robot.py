import magicbot
import wpilib
import ctre
import wpilib.drive
from robotpy_ext.common_drivers import navx

class MyRobot(magicbot.MagicRobot):


    def createObjects(self):
        self.init_drive_train()


    def init_drive_train(self):
        fl, bl, fr, br = (30, 40, 50, 10)  # practice bot
        br, fr, bl, fl = (1, 7, 2, 5)  # on competition robot

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



        self.robot_drive = wpilib.RobotDrive(self.fl_motor, self.bl_motor, self.fr_motor, self.br_motor)

    def teleopInit(self):
        pass

    def teleopPeriodic(self):
        self.robot_drive.arcadeDrive(self.joystick.getX(), self.joystick.getY())







if __name__ == '__main__':
    wpilib.run(MyRobot)