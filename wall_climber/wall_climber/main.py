"""
Main executable file.
"""

import time
import curses
import os
import io
import rclpy
from serial import SerialException
from wall_climber.motors import Motors
from wall_climber.robot import Robot
from wall_climber.teleop import Terminal
from wall_climber.sally_node import SallyNode
from wall_climber.transition import loop

def main_loop(terminal, buffer):
    """
    The main loop checks for operator input, reads the motor data, 
    computes force feedback, writes values to the motors, 
    and updates the display.
    """
    if os.name == "nt":
        port = "COM5"  # Windows
    else:
        port = "/dev/ttyUSB0"  # Linux
    
    node = SallyNode()
    robot = Robot(Motors(port=port, baud=57600), node)
    interface = Terminal(terminal, buffer, node)

    t = time.perf_counter()  # current time in seconds
    t0 = t  # start time for loop counter in seconds
    loops = 0  # loop counter for timing code

    while not interface.quit:

        dt = time.perf_counter() - t
        t += dt
        if dt > 1:  # Timeout
            continue

        if robot.mode == 0:
            interface.teleop(robot, dt)
        elif robot.mode == 1:
            loop(robot)

        if isinstance(interface, Terminal):
            interface.display()

        # robot.motors.read_velocity()
        robot.motors.read_angle()
        robot.motors.read_torque()

        rclpy.spin_once(node, timeout_sec=0)

        robot.update_state(node.get_orientation())
        robot.update_imu(node.get_acceleration())
        contact_forces = robot.get_contact_forces()
        opt_forces, _ = robot.get_optimized_forces()

        robot.force_control(contact_forces, opt_forces, dt)
        robot.motors.write_angle()
        robot.motors.write_velocity()
        robot.motors.write_torque()

        # For URDF force estimation
        node.publish_contact_forces(contact_forces)
        node.publish_optimized_forces(opt_forces)
        node.publish_joint_state(robot.motors.get())

        loops += 1
        if t - t0 > 0.5:
            robot.motors.read_voltage()
            robot.motors.read_temp()
            temp = max(m.temperature for m in robot.motors.get())
            volt = min(m.voltage for m in robot.motors.get())
            interface.status[0] = (
                f"--- {robot.motors.status} | "
                f"{(t - t0) / loops * 1000:.2f} ms | "
                f"{temp}°C | "
                f"Motor 9 {robot.lift_motors[0].angle} | "
                f"Motor 10 {robot.lift_motors[1].angle} | "
                f"{volt}V ---"
            )
            t0 = t
            loops = 0
            if temp > 70:  # AX limit is 75, XM limit is 80
                robot.motors.disable()
                for motor in robot.motors.get():
                    if motor.temperature > 70:
                        print(
                            f"TEMPERATURE OVERRIDE: motor {motor.id} at {motor.temperature}°C"
                        )


def main():
    """
    The main function starts the main loop
    """

    rclpy.init()

    buf = io.StringIO()
    try:
        curses.wrapper(lambda terminal: main_loop(terminal, buf))

    except SerialException:
        print("Disconnected")

    finally:
        os.system("cls" if os.name == "nt" else "clear")
        log = buf.getvalue()
        for s in log:
            print(s, end="")


if __name__ == "__main__":
    main()
