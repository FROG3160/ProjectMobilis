import math
from ctre import WPI_TalonFX

class DriveUnit:
    def __init__(
        self, gear_stages: list, motor_rpm: int, diameter: float, cpr: int
    ):
        """Constructs a DriveUnit object that stores data about the drive, gear stages, and wheel.
              The gear_stages is a list of tuples where each tuple defines one stage .e.g. (14, 28)
        Args:
            gear_stages (list): list of gear stages expressed as tuples of two integers e.g. [(10, 32), (9, 24)]
            motor_rpm (int): Maximum rpm of the attached motor
            diameter (float): Diameter of the attached wheel in meters
            cpr (int): Number of encoder counts per revolution
        """
        self.gearing =  math.prod(gear_stages)
        self.motor_rpm = motor_rpm
        self.cpr = cpr
        self.circumference = math.pi * diameter

    def speedToVelocity(self, speed: float) -> float:
        """Converts linear speed to Falcon velocity
        Args:
            speed (float): desired linear speed in meters per second
        Returns:
            float: velocity in encoder counts per 100ms
        """
        wheel_rotations_sec = speed / self.circumference
        motor_rotations_sec = wheel_rotations_sec / self.gearing
        ticks_per_sec = motor_rotations_sec * self.cpr
        return ticks_per_sec / 10

    def velocityToSpeed(self, velocity: float) -> float:
        """Converts Falcon velocity to linear speed
        Args:
            velocity (float): velocity in encoder counts per 100ms
        Returns:
            float: linear speed in meters per second
        """
        ticks_per_sec = velocity * 10
        motor_rotations_sec = ticks_per_sec / self.cpr
        wheel_rotations_sec = motor_rotations_sec * self.gearing
        return wheel_rotations_sec * self.circumference

def remap(val, OldMin, OldMax, NewMin, NewMax):
    """take a value in the old range and return a value in the new range"""
    return (((val - OldMin) * (NewMax - NewMin)) / (OldMax - OldMin)) + NewMin

class Rescale:
    def __init__(
        self,
        original_scale: tuple[float, float],
        new_scale: tuple[float, float],
        deadband: float = 0.0,
    ) -> None:
        """Class for transferring a value from one scale to another
        Args:
            original_scale (tuple[float, float]): the original scale the the given value will fall in.
                Expressed as a tuple: (minimum: float, maximum: float)
            new_scale (tuple[float, float]): the new scale the given value wil fall in.
                Expressed as a tuple: (minimum: float, maximum: float)
            deadband (0.0): A deadband value, if desired, defaults to 0.0.  If a deadband is specified,
                the deadband is subtracted from the original value and then matched to the new scale.  If
                the value is within the deadband, 0 is returned.
        """
        self.orig_min = original_scale[0] + deadband
        self.orig_max = original_scale[1] - deadband
        self.new_min, self.new_max = new_scale
        self.deadband = deadband

    def __call__(self, value):
        value = (
            math.copysign(abs(value) - self.deadband, value)
            if abs(value) > self.deadband
            else 0
        )
        return (
            ((value - self.orig_min) * (self.new_max - self.new_min))
            / (self.orig_max - self.orig_min)
        ) + self.new_min

    def setNewMax(self, value: float):
        self.new_max = value

class MotorUnit:
    
    motor = WPI_TalonFX
    
    def __init__(self, gear: list, p_value: float, i_value: float, d_value: float, wheel_diameter: float) -> None:
        self.gearing =  math.prod(gear)
        self.kp = p_value
        self.ki = i_value
        self.kd = d_value
        self.circumference = math.pi * wheel_diameter
    
    # Motor
    def speedToVelocity(self, speed: float) -> float:
        """Converts linear speed to Falcon velocity
        Args:
            speed (float): desired linear speed in meters per second
        Returns:
            float: velocity in encoder counts per 100ms
        """
        wheel_rotations_sec = speed / self.circumference
        motor_rotations_sec = wheel_rotations_sec / self.gearing
        ticks_per_sec = motor_rotations_sec * self.cpr
        return ticks_per_sec / 10

    def velocityToSpeed(self, velocity: float) -> float:
        """Converts Falcon velocity to linear speed
        Args:
            velocity (float): velocity in encoder counts per 100ms
        Returns:
            float: linear speed in meters per second
        """
        ticks_per_sec = velocity * 10
        motor_rotations_sec = ticks_per_sec / self.cpr
        wheel_rotations_sec = motor_rotations_sec * self.gearing
        return wheel_rotations_sec * self.circumference

        # Transmission ( Gear ratios )
        kModuleDriveGearing = [(14.0 / 50.0), (27.0 / 17.0), (15.0 / 45.0)]
        # End effector
        kWheelDiameter = 0.1000125  # 3 15/16 inches in meters
        # Motor Config ( For storing CAN IDs and PID values )
