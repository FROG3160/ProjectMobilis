from ctre import WPI_CANCoder, WPI_TalonFX, CANifier
import magicbot
from magicbot import feedback, tunable
import wpilib
import math
from wpilib import (DriverStation)
from components.drivetrain import SwerveModule, SwerveChassis
from wpimath.geometry import Translation2d
from wpimath.trajectory import TrapezoidProfile, TrapezoidProfileRadians
from wpimath.controller import (
    PIDController,
    HolonomicDriveController,
    ProfiledPIDControllerRadians,
)
from components.driverstation import FROGStick, FROGBoxGunner
from components.sensors import FROGGyro, FROGdar, FROGsonic, FROGColor


# robot characteristics
# we are specifying inches and dividing by 12 to get feet,
# giving us values that can be used with the fromFeet method
# to get a correct Translation2d object
trackwidth = 27.75 / 12  # feet between wheels side to side
wheelbase = 21.75 / 12  # feet between wheels front to back
kDeadzone = 0.2
joystickAxisDeadband = Rescale((-1, 1), (-1, 1), 0.15)
joystickTwistDeadband = Rescale((-1, 1), (-1, 1), 0.2)
visionTwistDeadband = Rescale((-1, 1), (-0.35, 0.35))


# visionDeadband = Rescale((-1,1), (.25, .25))

FIELD_ORIENTED = 0  # drive orientation
ROBOT_ORIENTED = 1
AUTO_DRIVE = 0  # drive mode
MANUAL_DRIVE = 1


class FROGbot(magicbot.MagicRobot):
    """
    Initialize components here.
    """


    gyro: FROGGyro
    # lidar: FROGdar
    swerveChassis: SwerveChassis

    swerveFrontLeft: SwerveModule
    swerveFrontRight: SwerveModule
    swerveBackLeft: SwerveModule
    swerveBackRight: SwerveModule

    # driverstation: DriverStation

    speedFactor = tunable(0.15)


    def createObjects(self):
        """Create motors and inputs"""
        # Swerve drive motors
        self.swerveFrontLeft_drive = WPI_TalonFX(11)
        self.swerveFrontRight_drive = WPI_TalonFX(12)
        self.swerveBackLeft_drive = WPI_TalonFX(13)
        self.swerveBackRight_drive = WPI_TalonFX(14)
        # Swerve steer motors
        self.swerveFrontLeft_steer = WPI_TalonFX(21)
        self.swerveFrontRight_steer = WPI_TalonFX(22)
        self.swerveBackLeft_steer = WPI_TalonFX(23)
        self.swerveBackRight_steer = WPI_TalonFX(24)
        # Swerve steer encoders (canifier)
        self.swerveFrontLeft_encoder = WPI_CANCoder(31)
        self.swerveFrontRight_encoder = WPI_CANCoder(32)
        self.swerveBackLeft_encoder = WPI_CANCoder(33)
        self.swerveBackRight_encoder = WPI_CANCoder(34)
        # Swerve module locations
        # TODO: move to swerveChassis?
        self.swerveFrontLeft_location = Translation2d.fromFeet(
            wheelbase / 2,
            trackwidth / 2,
        )
        self.swerveFrontRight_location = Translation2d.fromFeet(
            wheelbase / 2,
            -trackwidth / 2,
        )
        self.swerveBackLeft_location = Translation2d.fromFeet(
            -wheelbase / 2,
            trackwidth / 2,
        )
        self.swerveBackRight_location = Translation2d.fromFeet(
            -wheelbase / 2,
            -trackwidth / 2,
        )

        self.swerveFrontLeft_steerOffset = (
            132.451172  # 133.330078  # 136.143  # 26.807  # 21.796875  # 18.5449219 #13.008
        )
        self.swerveFrontRight_steerOffset = (
            -149.765625  # -150.380859  # -146.602
        )  # 178.0664  # 177.1875  # 174.023438 #171.914
        self.swerveBackLeft_steerOffset = (
            3.60351563  # 3.95507813  # -0.527  # 31.9921875  # 23.0273438  # 22.764
        )
        self.swerveBackRight_steerOffset = (
            -150.644531  # -150.117188  # -140.361
        )  # -43.33008  # -43.41797  # -43.242


        # self.vision_driverstation = DriverStation

        # config for saitek joystick
        # self.driveStick = FROGStick(0, 0, 1, 3, 2)
        # config for Logitech Extreme 3D
        self.driveStick = FROGStick(0, 0, 1, 2, 3)
        self.gunnerControl = FROGBoxGunner(1)

        self.field = wpilib.Field2d()
        # simulation already places field data in SmartDashboard
        # so we need to keep this from overwriting that data
        # during simulation
        if not self.isSimulation():
            wpilib.SmartDashboard.putData(self.field)

        # self.driverstation = DriverStation

        self.driverMode = False

        self.xOrig = self.yOrig = self.tOrig = 0

        self.vX = 0
        self.vY = 0
        self.vT = 0


    @feedback()
    def getControllerAxes(self):
        return self.xOrig, self.yOrig, self.tOrig

    @feedback()
    def getVX(self):
        return self.vX

    @feedback()
    def getVY(self):
        return self.vY

    @feedback()
    def getVT(self):
        return self.vT


    def teleopInit(self):
        """Called when teleop starts; optional"""
        self.swerveChassis.enable()
        self.autoDrive = False

    def teleopPeriodic(self):
        """Called on each iteration of the control loop"""

        # pessing that button resets the PID values for the swerve chasis 

        if self.driveStick.getRawButtonPressed(7):
            self.swerveChassis.configProfiledRotationController()
            self.swerveChassis.profiledRotationController.reset(
                math.radians(self.gyro.getOffsetYaw())
            )

        self.xOrig = joystickAxisDeadband(self.driveStick.getFieldForward())
        self.yOrig = joystickAxisDeadband(self.driveStick.getFieldLeft())

        # targetX = self.getSelectedTargetX()
        targetY = self.getSelectedTargetY()
        targetYaw = self.getSelectedTargetYaw()

        # ! If we are in autodrive and we have a target, use it
        # ! otherwise use manual drive
        # ! autodrive will need to not use field-oriented
        # ! we also have auto rotate to target.... call target lock?
        # ! for targeting the goal, we will most likely want to
        # !   allow the joystick to control X and Y, get rotation from
        # !   targeting.

        self.vX = self.vY = self.vT = 0
        targetAngle = None

        # determine twist/rotation
        if self.driveStick.getPOV() > -1:
            targetAngle = -(self.driveStick.getPOV() - 180)
        elif self.targetLock and targetYaw and not self.overrideTargeting:
            if self.firecontrol.isOnTarget():
                self.vT = 0
            else:
                self.vT = angleErrorToRotation(-targetYaw)
        else:
            new_twist = joystickTwistDeadband(
                self.driveStick.getFieldRotation()
            )
            self.vT = math.copysign(new_twist**3, new_twist)

        # determine linear velocities
        if (
            self.autoDrive
            and targetY
            and not self.overrideTargeting
            and self.objectTargeted == TARGET_CARGO
        ):
            targetY = targetY + 1
            self.vX = math.copysign(
                abs(
                    (((targetY + 1) / 2) * self.speedFactor)
                    + self.speedFactor / 2
                ),
                targetY,
            )
            self.driveMode = ROBOT_ORIENTED
            # self.driveController = AUTO_DRIVE
            self.vY = 0
        else:
            self.vX, self.vY = (
                math.copysign(self.xOrig**2, self.xOrig),
                math.copysign(self.yOrig**2, self.yOrig),
            )
            if self.driverMode:
                self.driveMode = ROBOT_ORIENTED
            else:
                self.driveMode = FIELD_ORIENTED

        if self.vX or self.vY or self.vT or targetAngle is not None:
            if self.driveMode == FIELD_ORIENTED:
                self.swerveChassis.field_oriented_drive(
                    self.vX, self.vY, self.vT, targetAngle
                )
            else:
                # robot oriented drive is from the perspective of the INTAKE
                # at the back of the robot, so we revers the X and Y
                self.swerveChassis.drive(
                    -self.vX, -self.vY, self.vT, targetAngle
                )
        else:
            self.swerveChassis.field_oriented_drive(0, 0, 0)

        if self.driveStick.getRawButtonPressed(3):
            self.gyro.resetGyro()
            self.gyro.starting_angle = 0
            self.swerveChassis.resetOdometry()
            self.swerveChassis.resetRemoteEncoders()
            self.swerveChassis.field_oriented_drive(0, 0, 0)

    def testInit(self):
        """Called when test mode starts; optional"""
        pass

    def testPeriodic(self):
        """Called on each iteration of the control loop"""
        self.led.ColorChangeBlue()
        pass


if __name__ == "__main__":
    wpilib.run(FROGbot)