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
import numpy as np

class Terminal:

    def __init__(self, terminal, buffer, sub):
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
        self.sub = sub
        
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
        self.heldB = True

        self.deadZone = 0.1
        self.joystick = False

    def teleop(self, robot, dt):
        """
        Check for user input and command robot appropriately
        :param robot: Robot to command
        :param dt: Time since last teleop update
        """
        c = self.terminal.getch()

        cont = self.sub.get_data()
        #print(cont)
        self.joystick = cont != None
        if self.joystick:
            self.LeftJoystickX = cont[0][0]
            self.LeftJoystickY = cont[0][1]
            self.RightJoystickX = cont[0][2]
            self.RightJoystickY = cont[0][3]
            self.RightTrigger = cont[0][4]
            self.LeftTrigger = cont[0][5]

            self.A = cont[1][0]
            self.B = cont[1][1]
            self.X = cont[1][3]
            self.Y = cont[1][4]
            self.LeftBumper = cont[1][6]
            self.RightBumper = cont[1][7]
            
        #print(self.B)
        
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
        self.execute(["+"], robot)

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
        
        # print(self.LeftTrigger, self.RightTrigger)
        if c == "+":
            pass
        if False and self.B:  # B Button
            print("WE HIT B")
            robot.stop()
            sys.stdout = self.stdout
            self.quit = True
        elif self.A and (np.abs(self.LeftJoystickY) > self.deadZone):
            robot.hold_two_drive(self.LeftJoystickY)
            print(t + "Hold Front and Drive" + str(self.LeftJoystickY))
        elif self.A:  # A Button
            robot.hold_two()
            print(t + "Hold Two")
        elif self.Y:
            robot.hold_four()
            print("Hold Four")
        elif self.LeftJoystickY > self.deadZone and self.X:  # X Button
            robot.drive(-1 * 0.6)
            print(t + "Drive Straight" + str(self.LeftJoystickY))
        elif self.LeftJoystickY > self.deadZone and self.X and self.A:  # X Button
            robot.hold_two_drive(-1*0.6)
            print(t + "Drive Straight" + str(self.LeftJoystickY))
        elif self.LeftJoystickY < -1*self.deadZone and self.X:  # X Button
            robot.drive(1)
            print(t + "Drive Straight" + str(self.LeftJoystickY))
        elif self.X:
            robot.set_straight()
            print("Set Straight")
        elif np.abs(self.LeftJoystickY) > self.deadZone or np.abs(self.LeftJoystickX) > self.deadZone:  # X Button
            robot.strafe_drive(self.LeftJoystickX, -1*self.LeftJoystickY)
            print(t + "Strafe-Drive " + str(self.LeftJoystickY))
        elif np.abs(self.LeftJoystickY) > self.deadZone > np.abs(self.LeftJoystickX):  # X Button
            robot.drive(-1*self.LeftJoystickY)
            print(t + "Drive " + str(self.LeftJoystickY))
        elif self.RightJoystickX < -0.05:
            robot.turn(0.4)
            print(t + "CCW")
        elif self.RightJoystickX > 0.05:
            robot.turn(-0.4)
            print(t + "CW")
        elif c == "p":  # quit
            robot.disable_steer()
            robot.stop()
            #sys.stdout = self.stdout
            self.quit = True
        elif c == " ":
            robot.stop()
            print(t + "Stop")
        elif c == "w":
            robot.drive(-0.6)
            print(t + "Forward")
        elif c == "s":
            robot.drive(0.6)
            print(t + "Reverse")
        elif c == "a":
            robot.strafe(0.4)
            print(t + "Left")
        elif c == "d":
            robot.strafe(-0.4)
            print(t + "Right")
        elif c == "e":
            robot.turn(0.4)
            print(t + "CCW")
        elif c == "q":
            robot.turn(-0.4)
            print(t + "CW")
        elif c == 'f':
            robot.hold_two_drive(0.5)
            print(t + "Hold Front and Drive Forward")
        elif c == 'b':
            robot.hold_two_drive(-0.5)
            print(t + "Hold Front and Drive Backward")
        elif c == '2':  # A Button
            robot.hold_two()
            print(t + "Hold Two")
        elif c == '4':
            robot.hold_four()
            print("Hold Four")
        elif c == '1':
            robot.set_straight()
            print("Set Straight")
        elif c == "r":
            if not robot.motors.opened:
                print(t + "Reconnect")
                robot.motors.connect()
            print(t + "Enable")
            robot.motors.enable()
        elif c == 'j':
            for i, id in enumerate(robot.drive_ids):
                if robot.motors.get(id).torque_mode:
                    robot.motors.enable(robot.drive_ids, velocity_mode=True, torque_mode=False)
                    for i, id in enumerate(robot.drive_ids):
                        robot.motors.get(id).goal_torque = 200
                        robot.motors.get(id).set_torque = 200
                        print(id, "GOING INTO VELOCITY MODE")
        elif c == 'k':
            for i, id in enumerate(robot.drive_ids):
                if robot.motors.get(id).velocity_mode:
                    robot.motors.enable(robot.drive_ids, velocity_mode=False, torque_mode=True)
                    for i, id in enumerate(robot.drive_ids):
                        robot.motors.get(id).goal_torque = 0
                        robot.motors.get(id).set_torque = 0
                        print(id, "GOING INTO TORQUE MODE")
        elif self.B and not self.heldB:
            self.heldB = True
            for i, id in enumerate(robot.drive_ids):
                if robot.motors.get(id).velocity_mode:
                    robot.motors.enable(robot.drive_ids, velocity_mode=False, torque_mode=True)
                    for i, id in enumerate(robot.drive_ids):
                        robot.motors.get(id).goal_torque = 0
                        robot.motors.get(id).set_torque = 0
                        print(id, "GOING INTO TORQUE MODE")
        elif (not self.B) and self.heldB:
            self.heldB = False
            for i, id in enumerate(robot.drive_ids):
                if robot.motors.get(id).torque_mode:
                    robot.motors.enable(robot.drive_ids, velocity_mode=True, torque_mode=False)
                    for i, id in enumerate(robot.drive_ids):
                        robot.motors.get(id).goal_torque = 200
                        robot.motors.get(id).set_torque = 200
                        print(id, "GOING INTO VELOCITY MODE")

        # print(robot.motors.get(5).set_velocity)
        elif self.joystick:
            robot.stop()

        if c == 'l':
            # robot.lift(1)
            robot.raise_elevator()
            print(t + "Elevator up")
        elif c == ';':
            # robot.lift(-1)
            print(t + "Elevator zero")
            robot.zero_elevator()
        elif c == "'":
            # robot.lift(-1)
            print(t + "Elevator down")
            robot.lower_elevator()
        elif self.LeftBumper:
            robot.raise_elevator()
            print(t + "Elevator up")
        elif self.RightBumper:
            # robot.lift(-1)
            print(t + "Elevator down")
            robot.lower_elevator()
        elif self.RightTrigger < 0:
            print(t + "Elevator zero")
            robot.zero_elevator()
        else:
            robot.lift(0)

        # robot.print_lift()
