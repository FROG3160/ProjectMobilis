import wpilib
from wpilib import Joystick
from .common import remap
from networktables import NetworkTables
from magicbot import feedback

class FROGStick(Joystick):
    """Extended class of wpilib.Joystick

    Returns:
        FROGStick: Custom Joystick class
        """

    DEADBAND = 0.15
    SPEED_DIVISOR = 1
    ROTATION_DIVISOR = 1.6
    ROTATION_MIN = 0
    ROTATION_MAX = 0.5
    DEBOUNCE_PERIOD = 0.5

    def __init__(
        self, port: int, xAxis: int = 1, yAxis: int = 2, rAxis = 3, tAxis = 4
    ) -> None:
        """Constructor for FROGStick

        :param port: The port on the Driver Station that the joystick is plugged into (0-5).
        :param xAxis: channel for the X axis
        :param yAxis: channel for the Y axis
        :param rAxis: channel for the rotation (twist) axis
        :param tAxis: channel for the throttle axis
        """

        super().__init__(port)
        self.setThrottleChannel(tAxis)
        self.setTwistChannel(rAxis)
        self.setXChannel(xAxis)
        self.setYChannel(yAxis)
        self.button_latest = {}
        self.timer = wpilib.Timer

        @feedback
        def getFieldForward(self):
            """Get's the joystick's Y axis and inverts it so pushing forward is positive and translates to chassis moving away from driver.

            Returns:
                float: -1 to 1
            """
            #inverts the joystick's Y axis so pushing
            #forward is positive and pulling back is
            #negative
            return -self.getY()

        @feedback
        def getFieldLeft(self):
            """Get's the jpystick's X axis and inverts it so pushing left is positive and translates to chassis moving to the left of the driver.

            Returns:
                float: -1 to 1
            """
        #inverts the joystick's X axis so pushing
        #left is positive and pushing right is negative
        return -self.getX()

        