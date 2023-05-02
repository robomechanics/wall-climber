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
steer_offsets = (-160, -20, 0, 0)

elevator_left_offset = -253
elevator_right_offset = -406

class Robot:
    def __init__(self, motors):
        self.drive_motors = motors.add(drive_ids, 'XM430-W210-T', mirror=(5, 7))
        self.steer_motors = motors.add(steer_ids, 'XM430-W210-T',
                                       offset={steer_ids[i]: steer_offsets[i] for i in range(4)})
        self.lift_motors = motors.add(lift_ids, 'XM430-W210-T', mirror=(9,))
        motors.enable(drive_ids, torque_mode=True)
        motors.enable(steer_ids, velocity_mode=False)
        motors.enable(lift_ids, velocity_mode=False)
        self.drive_ids = drive_ids
        self.motors = motors
        for i, id in enumerate(lift_ids):
            self.motors.get(id).goal_torque = 400
            self.motors.get(id).set_torque = 400

        for i, id in enumerate(drive_ids):
            self.motors.get(id).goal_torque = 0
            self.motors.get(id).set_torque = 0

    def drive_torque(self, v):
        """
        Drive forward/reverse
        :param v: Velocity scaled from -1 (reverse) to 1 (forward)
        """
        for i, id in enumerate(drive_ids):
            # self.motors.get(id).set_velocity = v * self.motors.get(id).speed
            if ((id in (5, 6)) and (v < 0)) or (id in (7, 8) and (v > 0)):
                self.motors.get(id).set_torque = v * self.motors.get(id).stall * 0 / 4
            else:
                self.motors.get(id).set_torque = v * self.motors.get(id).stall / 2
        for i, id in enumerate(steer_ids):
            self.motors.get(id).set_angle = 0

    def drive_vel(self, v):
        """
        Drive forward/reverse
        :param v: Velocity scaled from -1 (reverse) to 1 (forward)
        """
        for i, id in enumerate(drive_ids):
            self.motors.get(id).set_velocity = v * self.motors.get(id).speed
            # self.motors.get(id).set_torque = v * self.motors.get(id).stall / 2
        for i, id in enumerate(steer_ids):
            self.motors.get(id).set_angle = 0

    def drive(self, v):
        if self.motors.get(5).torque_mode:
            self.drive_torque(v)
        else:
            self.drive_vel(v)

    def strafe_torque(self, v):
        """
        Strafe left/right
        :param v: Velocity scaled from -1 (right) to 1 (left)
        """
        for i, id in enumerate(drive_ids):
            if id == 4:
                self.motors.get(id).set_torque = -1 * v * self.motors.get(id).stall / 2
            else:
                self.motors.get(id).set_torque = v * self.motors.get(id).stall / 2
        for i, id in enumerate(steer_ids):
            if id == 7:
                self.motors.get(id).set_angle = -90  # * np.sign(v)
            else:
                self.motors.get(id).set_angle = 90  # * np.sign(v)

    def strafe_vel(self, v):
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

    def strafe(self, v):
        if self.motors.get(5).torque_mode:
            self.strafe_torque(v)
        else:
            self.strafe_vel(v)

    def disable_steer(self):
        for i, id in enumerate(steer_ids):
            self.motors.get(id).set_torque = 0

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
                self.motors.get(id).set_torque = v * self.motors.get(id).stall / 2
            else:
                self.motors.get(id).set_torque = 0

    def set_straight(self):
        for i, id in enumerate(steer_ids):
            self.motors.get(id).set_angle = 0

    def lift(self, v):
        for i, id in enumerate(lift_ids):
            self.motors.get(id).set_velocity = v * self.motors.get(id).speed

    def stop_lift(self):
        for i, id in enumerate(lift_ids):
            self.motors.get(id).set_torque = 0

    def strafe_drive(self, x, y):
        v = np.sqrt(x ** 2 + y ** 2)
        angle = np.degrees(np.arctan2(x, y))
        if abs(angle) > 135:
            angle -= 180 * np.sign(angle)
            v *= -1
        for i, id in enumerate(steer_ids):
            self.motors.get(id).set_angle = angle
        for i, id in enumerate(drive_ids):
            self.motors.get(id).set_torque = v * self.motors.get(id).stall / 2

    def print_lift(self):
        for i, id in enumerate(lift_ids):
            print(id + self.motors.get(id).angle)
    def turn_torque(self, v):
        """
        Turn CW/CCW
        :param v: Angular velocity scaled from 1 (CW) to 1 (CCW)
        """
        drive_dirs = (-1, 1, -1, 1)
        for i, id in enumerate(drive_ids):
            self.motors.get(id).set_torque = drive_dirs[i] * v * self.motors.get(id).stall / 1
        steer_dirs = (1, -1, -1, 1)
        for i, id in enumerate(steer_ids):
            self.motors.get(id).set_angle = steer_dirs[i] * 60

    def turn_vel(self, v):
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

    def turn(self, v):
        if self.motors.get(5).torque_mode:
            self.turn_torque(v)
        else:
            self.turn_vel(v)

    def stop(self):
        """
        Stop all motors
        """
        for id in drive_ids:
            self.motors.get(id).set_torque = 0
            self.motors.get(id).set_velocity = 0
        # for id in lift_ids:
        #     self.motors.get(id).set_torque = 0
        # for id in steer_ids:
        #     self.motors.get(id).set_angle = self.motors.get(id).angle

    def zero_elevator(self):
        self.lift_motors[0].set_angle = 13.4 + elevator_left_offset + 100
        self.lift_motors[1].set_angle = 14.1 + elevator_right_offset + 100

    def raise_elevator(self):
        self.lift_motors[0].set_angle = -613.3 + elevator_left_offset + 250
        self.lift_motors[1].set_angle = -590.1 + elevator_right_offset + 250

    def lower_elevator(self):
        self.lift_motors[0].set_angle = 194.8 + elevator_left_offset + 100
        self.lift_motors[1].set_angle = 156.4 + elevator_right_offset + 100
