import math
from magicbot import feedback, tunable
from navx import AHRS
from wpimath.geometry import Rotation2d

class FROGGyro:

    starting_angle = tunable(0.0)

    def __init__(self):
        # TODO Make sure if we need this.
        self.gyro = AHRS.create_spi()
        self.offset = 0
        # self.field_heading = 360-242
        # self.gyro.reset()
        # self.gyro.setAngleAdjustment(-self.field_heading)

    @feedback()
    def getYaw(self):
        # returns gyro heading +180 to -180 degrees
        return -self.gyro.getYaw()

    def setOffset(self, offset):
        self.offset = offset

    @feedback()
    def getRotationDPS(self):
        return self.gyro.getRate()

    @feedback()
    def getOffsetYaw(self):
        chassisYaw = self.getYaw()
        fieldYaw = Rotation2d.fromDegrees(chassisYaw + self.starting_angle)
        return math.degrees(math.atan2(fieldYaw.sin(), fieldYaw.cos()))

    def resetGyro(self):
        # sets yaw reading to 0
        self.setAngleAdjustment(self.starting_angle)
        self.gyro.reset()

    def execute(self):
        pass

    @feedback
    def getAngle(self):
        return self.gyro.getAngle()

    @feedback
    def getAngleConstrained(self):
        angle = self.getAngle()
        return math.degrees(math.atan2(math.sin(angle), math.cos(angle)))

    def setAngleAdjustment(self, angle):
        self.gyro.setAngleAdjustment(angle)

    @feedback()
    def getRadiansCCW(self):
        return math.radians(self.gyro.getYaw())

    @feedback()
    def getAngleAdjustment(self):
        return self.gyro.getAngleAdjustment()