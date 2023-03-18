"""
Evaluate current robot state and modify setpoints using forward and inverse kinematics
"""

import numpy as np

# Wheel numbering
FL = 1  # Front left
FR = 2  # Front right
RL = 3  # Rear left
RR = 4  # Rear right

drive_ids = (5, 6, 7, 8)
steer_ids = (1, 2, 3, 4)
lift_ids = (9, 10)
steer_offsets = (-17, 0, -17, 0)


class Robot:
    def __init__(self, motors):
        self.drive_motors = motors.add(drive_ids, 'XM430-W210-T', mirror=(5, 7))
        self.steer_motors = motors.add(steer_ids, 'XM430-W210-T',
                                       offset={steer_ids[i]: steer_offsets[i] for i in range(4)})
        self.lift_motors = motors.add(lift_ids, 'XM430-W210-T', mirror=(9,))
        motors.enable(drive_ids, velocity_mode=True)
        motors.enable(steer_ids, velocity_mode=False)
        motors.enable(lift_ids, velocity_mode=True)
        self.motors = motors
        for i, id in enumerate(lift_ids):
            self.motors.get(id).goal_torque = 100
            self.motors.get(id).set_torque = 100

    def drive(self, v):
        """
        Drive forward/reverse
        :param v: Velocity scaled from -1 (reverse) to 1 (forward)
        """
        for i, id in enumerate(drive_ids):
            self.motors.get(id).set_velocity = v * self.motors.get(id).speed
        for i, id in enumerate(steer_ids):
            self.motors.get(id).set_angle = 0

    def strafe(self, v):
        """
        Strafe left/right
        :param v: Velocity scaled from -1 (right) to 1 (left)
        """
        for i, id in enumerate(drive_ids):
            if id == 4:
                self.motors.get(id).set_velocity = -1 * v * self.motors.get(id).speed
            else:
                self.motors.get(id).set_velocity = v * self.motors.get(id).speed
        for i, id in enumerate(steer_ids):
            if id == 7:
                self.motors.get(id).set_angle = -90  # * np.sign(v)
            else:
                self.motors.get(id).set_angle = 90  # * np.sign(v)

    def disable_steer(self):
        for i, id in enumerate(steer_ids):
            self.motors.get(id).set_torque = 0;

    def hold_four(self):
        for i, id in enumerate(steer_ids):
            if id in [7, 8, 10]:
                self.motors.get(id).set_angle = -90
            else:
                self.motors.get(id).set_angle = 90

    def hold_two(self):
        self.motors.get(1).set_angle = 90
        self.motors.get(2).set_angle = -90
        self.motors.get(3).set_angle = 0
        self.motors.get(4).set_angle = 0

    def hold_two_drive(self, v):
        self.hold_two()
        for i, id in enumerate(drive_ids):
            if id in [7, 8]:
                self.motors.get(id).set_velocity = v * self.motors.get(id).speed
            else:
                self.motors.get(id).set_velocity = 0

    def set_straight(self):
        for i, id in enumerate(steer_ids):
            self.motors.get(id).set_angle = 0

    def lift(self, v):
        for i, id in enumerate(lift_ids):
            self.motors.get(id).set_velocity = v * self.motors.get(id).speed

    def strafe_drive(self, x, y):
        v = np.sqrt(x ** 2 + y ** 2)
        angle = np.degrees(np.arctan2(x, y))
        if abs(angle) > 135:
            angle -= 180 * np.sign(angle)
            v *= -1
        for i, id in enumerate(steer_ids):
            self.motors.get(id).set_angle = angle
        for i, id in enumerate(drive_ids):
            self.motors.get(id).set_velocity = v * self.motors.get(id).speed

    def print_lift(self):
        for i, id in enumerate(lift_ids):
            print(id + self.motors.get(id).angle)
    def turn(self, v):
        """
        Turn CW/CCW
        :param v: Angular velocity scaled from 1 (CW) to 1 (CCW)
        """
        drive_dirs = (-1, 1, -1, 1)
        for i, id in enumerate(drive_ids):
            self.motors.get(id).set_velocity = drive_dirs[i] * v * self.motors.get(id).speed
        steer_dirs = (1, -1, -1, 1)
        for i, id in enumerate(steer_ids):
            self.motors.get(id).set_angle = steer_dirs[i] * 60

    def stop(self):
        """
        Stop all motors
        """
        for id in drive_ids:
            self.motors.get(id).set_velocity = 0
        for id in lift_ids:
            self.motors.get(id).set_velocity = 0
        for id in steer_ids:
            self.motors.get(id).set_angle = self.motors.get(id).angle
