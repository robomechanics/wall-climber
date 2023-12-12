#!/usr/bin/env python3

from motors import Motors
from robot import Robot
from teleop import Terminal
from serial import SerialException
from teleop import Joystick
import io
import time
import curses           # pip install windows-curses
import os


def main_loop(terminal, buffer):
    interface = Terminal(terminal, buffer)
    #interface = Joystick(terminal, buffer)
    print(os.name)
    if os.name == 'nt':
        port = "COM5"          # Windows
    else:
        port = "/dev/ttyUSB0"   # Linux
    robot = Robot(Motors(port=port, baud=57600))

    t = time.perf_counter()     # current time in seconds
    t0 = t                      # start time for loop counter in seconds
    loops = 0                   # loop counter for timing code
    while not interface.quit:

        dt = time.perf_counter() - t
        t += dt
        if dt > 1:  # Timeout
            continue

        interface.teleop(robot, dt)
        interface.display()

        robot.motors.read_angle()
        robot.motors.write_angle()
        robot.motors.write_velocity()
        robot.motors.write_torque()

        loops += 1
        if t - t0 > 0.25:
            robot.motors.read_voltage()
            robot.motors.read_temp()
            temp = max(m.temperature for m in robot.motors.get())
            volt = min(m.voltage for m in robot.motors.get())
            interface.status[0] = f"--- {robot.motors.status} | " \
                                  f"{(t - t0) / loops * 1000:.2f} ms | " \
                                  f"{temp}°C | " \
                                  f"Motor 9 {robot.lift_motors[0].angle} | " \
                                  f"Motor 10 {robot.lift_motors[1].angle} | " \
                                  f"{volt}V ---"
            t0 = t
            loops = 0
            if temp > 70:   # AX limit is 75, XM limit is 80
                robot.motors.disable()
                for motor in robot.motors.get():
                    if motor.temperature > 70:
                        print(f"TEMPERATURE OVERRIDE: motor {motor.id} at {motor.temperature}°C")


if __name__ == "__main__":
    buf = io.StringIO()
    try:
        curses.wrapper(lambda terminal: main_loop(terminal, buf))
        os.system('cls' if os.name == 'nt' else 'clear')
        log = buf.getvalue()
        for s in log:
            print(s, end="")
    except SerialException:
        print("Disconnected")
