"""
Parse user input to control the robot

" " = stop
"w" = drive forward
"a" = strafe left
"s" = strafe right
"d" = drive reverse
"e" = turn clockwise
"q" = turn counter-clockwise

"m 5 30" = set motor 5 angle to 30 deg
"v 10 10" = set motor 10 angular velocity to 10 deg/s
"t 3 100" = set motor 3 torque limit to 100 N-mm

"r" = reconnect/re-enable motors
"p" = print log to output and exit
"""

import sys
import time
import controller
import numpy as np


class Joystick:

    def __init__(self, terminal, buffer):
        self.terminal = terminal
        self.terminal.nodelay(True)
        self.joystick = controller.XboxController()
        self.command = ""
        self.command_text = ""
        self.status = ["Loading", "", ""]
        self.t = 0
        self.quit = False
        self.stdout = sys.stdout
        self.i = 0

        self.LeftJoystickY = 0
        self.LeftJoystickX = 0
        self.RightJoystickY = 0
        self.RightJoystickX = 0
        self.LeftTrigger = 0
        self.RightTrigger = 0
        self.LeftBumper = 0
        self.RightBumper = 0
        self.A = 0
        self.X = 0
        self.Y = 0
        self.B = 0

        sys.stdout = self.buffer = buffer

    def teleop(self, robot, dt):
        """
        Check for user input and command robot appropriately
        :param robot: Robot to command
        :param dt: Time since last teleop update
        """
        # c = self.terminal.getch()
        c = self.joystick.read()
        self.stdout.write(str(c) + "\n")
        if c != -1:
            # c = chr(c)
            self.t = 0
            [self.LeftJoystickX, self.LeftJoystickY, self.RightJoystickX, self.RightJoystickY,
             self.A, self.X, self.B, self.Y,
             self.LeftBumper, self.RightBumper, self.LeftTrigger, self.RightTrigger] = c
        self.t += dt
        self.execute(robot)

    def display(self):
        """
        Update the user interface display
        """
        buffer = self.buffer.getvalue()
        n = sum([1 if s else 0 for s in self.status])
        self.terminal.erase()
        y, x = self.terminal.getmaxyx()
        lines = buffer[-10000:].split("\n")[-y + n:-1]
        for i in range(y - 1 - n - len(lines)):
            self.terminal.addstr("\n")
        for line in lines:
            self.terminal.addstr(line[:x - 1] + "\n")
        for i in range(len(self.status)):
            if self.status[i]:
                self.terminal.addstr((self.status[i].replace("\n", " \\ ") + "\n")[:x - 1])
        self.terminal.addstr(("> " + self.command_text)[:x - 1])
        if self.i < len(buffer):
            self.stdout.write(buffer[self.i:])
            self.i = len(buffer)

    def execute(self, robot):
        """
               Execute the given command
               :param robot: Robot to command
               """
        t = f'{time.perf_counter():.3f}: '
        self.stdout.write(str(self.LeftJoystickY))
        # try:
        #     if c in "mvt":
        #         s = command[1:].replace("=", " ").strip(" \t=").split(" ")
        #         id = int(s[0].strip(" \t="))
        #         val = float(s[1].strip(" \t="))
        #         if id not in robot.motors.motors_by_id:
        #             return
        #         if c == "m":
        #             print(t + f"Setting motor {id} to angle {val} degrees")
        #             robot.motors.motors_by_id[id].set_angle = val
        #         elif c == "v":
        #             print(t + f"Setting motor {id} to speed {val} degrees per second")
        #             robot.motors.motors_by_id[id].set_velocity = val
        #         elif c == "t":
        #             print(t + f"Setting motor {id} to torque {val} Newton millimeters")
        #             robot.motors.motors_by_id[id].set_torque = val
        # except (ValueError, IndexError):
        #     print(t + f"Invalid command: {command}")
        #     pass
        if self.B:  # B Button
            robot.stop()
            sys.stdout = self.stdout
            self.quit = True
        elif self.A:  # A Button
            robot.hold_two()
            print(t + "Stop")
        elif self.Y:
            robot.hold_four()
            print("Hold Four")
        elif self.X:
            robot.set_straight()
            print("Set Straight")
        elif np.abs(self.LeftJoystickY) > 0.06 or np.abs(self.LeftJoystickX) > 0.06:  # X Button
            robot.strafe_drive(self.LeftJoystickX, self.LeftJoystickY)
            print(t + "Strafe-Drive " + str(self.LeftJoystickY))
        elif np.abs(self.LeftJoystickY) > 0.06 > np.abs(self.LeftJoystickX):  # X Button
            robot.drive(self.LeftJoystickY)
            print(t + "Drive " + str(self.LeftJoystickY))
        # elif self.LeftJoystickY < -0.05:
        #     robot.drive(0.4)
        #     print(t + "Reverse")
        # elif self.LeftJoystickX > 0.05:
        #     robot.strafe(-0.4)
        #     print(t + "Left")
        # elif self.LeftJoystickX < -0.05:
        #     robot.strafe(0.4)
        #     print(t + "Right")
        elif self.RightJoystickX < -0.05:
            robot.turn(0.4)
            print(t + "CCW")
        elif self.RightJoystickX > 0.05:
            robot.turn(-0.4)
            print(t + "CW")
        # elif c == "r":
        #     if not robot.motors.opened:
        #         print(t + "Reconnect")
        #         robot.motors.connect()
        #     print(t + "Enable")
        #     robot.motors.enable()
        else:
            robot.stop()


class Terminal:

    def __init__(self, terminal, buffer):
        """
        Initialize terminal as user interface
        :param terminal: Curses window object
        """
        self.terminal = terminal
        self.terminal.nodelay(True)
        self.command = ""
        self.command_text = ""
        self.status = ["Loading", "", ""]
        self.t = 0
        self.quit = False
        self.stdout = sys.stdout
        self.i = 0
        sys.stdout = self.buffer = buffer

    def teleop(self, robot, dt):
        """
        Check for user input and command robot appropriately
        :param robot: Robot to command
        :param dt: Time since last teleop update
        """
        c = self.terminal.getch()
        if c != -1:
            c = chr(c)
            self.t = 0
            if self.command or c in "mvt":  # continue longer command
                if c == "\n":  # submit command
                    self.execute(self.command, robot)
                    self.command = ""
                    self.command_text = ""
                elif c == "\b":  # backspace
                    self.command = self.command[:-1]
                else:  # continue command
                    self.command += c
            elif c != "\n":  # start new command
                self.execute(c, robot)
                self.command = ""
                self.command_text = c
        self.t += dt
        if self.command:
            self.command_text = self.command
        elif self.t > 0.5:
            self.command_text = ""

    def display(self):
        """
        Update the user interface display
        """
        buffer = self.buffer.getvalue()
        n = sum([1 if s else 0 for s in self.status])
        self.terminal.erase()
        y, x = self.terminal.getmaxyx()
        lines = buffer[-10000:].split("\n")[-y + n:-1]
        for i in range(y - 1 - n - len(lines)):
            self.terminal.addstr("\n")
        for line in lines:
            self.terminal.addstr(line[:x - 1] + "\n")
        for i in range(len(self.status)):
            if self.status[i]:
                self.terminal.addstr((self.status[i].replace("\n", " \\ ") + "\n")[:x - 1])
        self.terminal.addstr(("> " + self.command_text)[:x - 1])
        if self.i < len(buffer):
            self.stdout.write(buffer[self.i:])
            self.i = len(buffer)

    def execute(self, command, robot):
        """
        Execute the given command
        :param command: Command string
        :param robot: Robot to command
        """
        c = command[0]
        t = f'{time.perf_counter():.3f}: '
        try:
            if c in "mvt":
                s = command[1:].replace("=", " ").strip(" \t=").split(" ")
                id = int(s[0].strip(" \t="))
                val = float(s[1].strip(" \t="))
                if id not in robot.motors.motors_by_id:
                    return
                if c == "m":
                    print(t + f"Setting motor {id} to angle {val} degrees")
                    robot.motors.motors_by_id[id].set_angle = val
                elif c == "v":
                    print(t + f"Setting motor {id} to speed {val} degrees per second")
                    robot.motors.motors_by_id[id].set_velocity = val
                elif c == "t":
                    print(t + f"Setting motor {id} to torque {val} Newton millimeters")
                    robot.motors.motors_by_id[id].set_torque = val
        except (ValueError, IndexError):
            print(t + f"Invalid command: {command}")
            pass
        if c == "p":  # quit
            sys.stdout = self.stdout
            self.quit = True
        elif c == " ":
            robot.stop()
            print(t + "Stop")
        elif c == "w":
            robot.drive(-0.4)
            print(t + "Forward")
        elif c == "s":
            robot.drive(0.4)
            print(t + "Reverse")
        elif c == "a":
            robot.strafe(-0.4)
            print(t + "Left")
        elif c == "d":
            robot.strafe(0.4)
            print(t + "Right")
        elif c == "q":
            robot.turn(0.4)
            print(t + "CCW")
        elif c == "e":
            robot.turn(-0.4)
            print(t + "CW")
        elif c == "r":
            if not robot.motors.opened:
                print(t + "Reconnect")
                robot.motors.connect()
            print(t + "Enable")
            robot.motors.enable()
