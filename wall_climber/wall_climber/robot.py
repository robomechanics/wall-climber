"""
Evaluate current robot state and modify setpoints using forward and inverse kinematics
"""

import numpy as np
import matplotlib.pyplot as plt
from qpsolvers import solve_qp
from scipy.linalg import null_space
from rcl_interfaces.msg import ParameterDescriptor

class Robot:
    def __init__(self, motors, node=None):
        # Declare parameters
        params = [
            ("steer_ids", [1, 2, 3, 4], "Steer motors ids (FL, FR, RL, RR)"),
            ("drive_ids", [5, 6, 7, 8], "Drive motors ids (FL, FR, RL, RR)"),
            ("lift_ids", [9, 10], "Lift motors ids (L, R)"),
            ("steer_offsets", [0.0, -20.0, 45.0, -45.0], "Steering motors' angle offset (degrees CW)"),
            ("lift_offsets", [0.0, 0.0], "Elevator angle offset(L, R) (degrees)"),
            ("lift_lowered", 161.0, "Elevator minimum height (degrees)"),
            ("lift_zero", 0.0, "Elevator ground height (degrees)"),
            ("lift_raised", -464.0, "Elevator maximum height (degrees)"),
            ("mass", 8.3, "Robot mass w/o pXRF (kg)"),
            ("magnet_force", 50.0, "Magnetic adhesion force of each wheel (N)"),
            ("radius", 0.025, "Robot wheel radius (m)"),
            ("height", 0.064, "Robot height (m)"),
            ("width", 0.317, "Robot width (m)"),
            ("length", 0.433, "Robot length (m)"),
            ("friction", 0.9, "Wheel tape friction coefficient"),
            ("max_torque", 2.0, "Maximum motor torque (Nm)"),
            ("regularization", 1.0e-3, "For controller regularization H (1/N)"),
            ("kp", 70.0, "Proportional gain for PD controller"),
            ("kd", 10.0, "Derivative gain for PD controller"),
        ]
        if node:
            for param_name, param_value, param_desc in params:
                node.declare_parameter(param_name, param_value,
                                    ParameterDescriptor(description=param_desc))

        # Load parameters
        param_vals = {}
        for name, default_val, _ in params:
            if node:
                param_vals[name] = node.get_parameter(name).value
            else:
                param_vals[name] = default_val

        self.steer_ids = param_vals["steer_ids"]
        self.drive_ids = param_vals["drive_ids"]
        self.lift_ids = param_vals["lift_ids"]
        self.steer_offsets = param_vals["steer_offsets"]
        self.lift_offsets = param_vals["lift_offsets"]
        self.lift_lowered = param_vals["lift_lowered"]
        self.lift_zero = param_vals["lift_zero"]
        self.lift_raised = param_vals["lift_raised"]
        self.mass = param_vals["mass"]
        self.magnet_force = param_vals["magnet_force"]
        self.radius = param_vals["radius"]
        self.height = param_vals["height"]
        self.width = param_vals["width"]
        self.length = param_vals["length"]
        self.friction = param_vals["friction"]
        self.max_torque = param_vals["max_torque"]
        self.regularization = param_vals["regularization"]
        self.kp = param_vals["kp"]
        self.kd = param_vals["kd"]

        # Configure motors
        self.drive_motors = motors.add(self.drive_ids, "XM430-W210-T", mirror=(5, 7))
        self.steer_motors = motors.add(
            self.steer_ids,
            "XM430-W210-T",
            offset={self.steer_ids[i]: self.steer_offsets[i] for i in range(4)}
        )
        self.lift_motors = motors.add(
            self.lift_ids,
            "XM430-W210-T",
            mirror=(9,),
            offset={self.lift_ids[i]: self.lift_offsets[i] for i in range(2)}
        )
        motors.enable(self.drive_ids, torque_mode=True)
        motors.enable(self.steer_ids, velocity_mode=False)
        motors.enable(self.lift_ids, velocity_mode=False)
        self.motors = motors
        self.steer_fl, self.steer_fr, self.steer_rl, self.steer_rr = self.steer_motors
        self.drive_fl, self.drive_fr, self.drive_rl, self.drive_rr = self.drive_motors

        # Initialize state variables
        self.acceleration = [0, 0, 0]
        self.diff_prev = np.zeros((len(self.drive_ids) * 3, 1))
        self.mode = 0   # 0 is teleop, 1 is transition
        self.force_control_on = False
        self.torque_mode = False

        # Initialize data logging
        self.n = len(self.motors.get())
        self.time = []
        self.torques = [[0 for i in range(10)] for j in range(self.n)]
        self.velocities = [[0 for i in range(10)] for j in range(self.n)]
        self.orientation = [[0 for i in range(10)] for j in range(3)]

        # Initialize motor torques in Nmm
        for motor in self.lift_motors:
            motor.goal_torque = 400
            motor.set_torque = 400
        for motor in self.drive_motors:
            motor.goal_torque = 0
            motor.set_torque = 0
    def set_torque_mode(self):
        """
        Set a boolean to the velocity or torque mode, 
        transition motors to torque mode and stop all motors.
        """
        if self.torque_mode:
            return
        self.motors.enable(self.drive_ids, velocity_mode=False, torque_mode=True)
        self.torque_mode = True
        for motor in self.drive_motors:
            motor.set_torque = 0
            motor.goal_torque = 0
            motor.set_velocity = 0
            motor.goal_velocity = 0

    def set_velocity_mode(self):
        """
        Set a boolean to the velocity or torque mode, 
        transition motors to velocity mode.
        """
        if not self.torque_mode:
            return
        else:
            self.motors.enable(self.drive_ids, velocity_mode=True, torque_mode=False)
            self.torque_mode = False
            for motor in self.drive_motors:
                motor.set_torque = 200
                motor.goal_torque = 200
                motor.set_velocity = 0
                motor.goal_velocity = 0

    def get_pitch(self):
        """
        calculates the pitch of the robot
        """
        return sum(self.orientation[0]) / len(self.orientation[0])

    def drive_torque(self, speed):
        """
        Drive forward/reverse
        :param speed: torque scaled from -1 (reverse) to 1 (forward)
        """
        for motor in self.drive_motors:
            # motor.goal_velocity = v * motor.speed
            if ((motor in [self.drive_fl, self.drive_fr]) and (speed < 0)) or (motor in [self.drive_rl, self.drive_rr] and (speed > 0)):
                motor.set_torque = speed * motor.stall * 0 / 4
            else:
                motor.set_torque = speed * motor.stall / 2
        for motor in self.steer_motors:
            motor.set_angle = 0
    def drive_vel(self, speed):
        """
        Drive forward/reverse
        :param speed: Velocity scaled from -1 (reverse) to 1 (forward)
        """
        for motor in self.drive_motors:
            motor.goal_velocity = speed * motor.speed
        for motor in self.steer_motors:
            motor.set_angle = 0

    def drive(self, speed):
        """
        Drive forward/reverse in the current operating mode (velocity or torque)
        :param speed: Velocity or torque scaled from -1 (reverse) to 1 (forward)
        """
        if self.drive_fl.torque_mode:
            self.drive_torque(speed)
        else:
            self.drive_vel(speed)

    def strafe_torque(self, speed):
        """
        Strafe left/right
        :param speed: Torque scaled from -1 (right) to 1 (left)
        """
        for motor in self.steer_motors:
            if motor in [self.steer_fl, self.steer_rr]:
                motor.set_angle = 90
            else:
                motor.set_angle = -90
        for motor in self.drive_motors:
            if motor in [self.drive_fl, self.drive_rr]:
                motor.set_torque = speed * motor.stall / 2
            else:
                motor.set_torque = -speed * motor.stall / 2

    def strafe_vel(self, speed):
        """
        Strafe left/right
        :param speed: Velocity scaled from -1 (right) to 1 (left)
        """
        for motor in self.steer_motors:
            if motor in [self.steer_fl, self.steer_rr]:
                motor.set_angle = 90
            else:
                motor.set_angle = -90
        for motor in self.drive_motors:
            if motor in [self.drive_fl, self.drive_rr]:
                motor.goal_velocity = speed * motor.speed
            else:
                motor.goal_velocity = -speed * motor.speed

    def strafe(self, speed):
        """
        Strafe left/right in the current operating mode (velocity or torque)
        :param speed: Velocity or torque scaled from -1 (right) to 1 (left)
        """
        if self.torque_mode:
            self.strafe_torque(speed)
        else:
            self.strafe_vel(speed)

    def disable_steer(self):
        """
        Set steer motors output to 0
        """
        for motor in self.steer_motors:
            motor.set_torque = 0

    def hold_four(self):
        """
        Set all motors orientation sideways
        """
        for motor in self.steer_motors:
            if motor in [self.steer_fl, self.steer_rr]:
                motor.set_angle = 90
            else:
                motor.set_angle = -90

    def hold_two(self):
        """
        Set front motors orientation sideways
        """
        self.steer_fl.set_angle = 90
        self.steer_fr.set_angle = -90
        self.steer_rl.set_angle = 0
        self.steer_rr.set_angle = 0

    def hold_two_drive(self, speed):
        """
        Break free from previous surface
        with front wheels turning sideways
        rear wheels will drive backwards to pull away
        :param speed: Velocity scaled from -1 (right) to 1 (left)
        """
        self.hold_two()
        
        if self.torque_mode:
            print("torque mode hold two")
            for motor in self.drive_motors:
                if motor in [self.drive_rl, self.drive_rr]:
                    motor.set_torque = speed * motor.stall / 2
                else:
                    motor.set_torque = 0
        else:
            print("vel mode hold two")
            for motor in self.drive_motors:
                if motor in [self.drive_rl, self.drive_rr]:
                    motor.goal_velocity = speed * motor.speed
                else:
                    motor.goal_velocity = 0

    def set_straight(self):
        """
        Set all wheels orientation front
        """
        for motor in self.steer_motors:
            motor.set_angle = 0

    def lift(self, speed):
        """
        Set elevator motors lifting the elevator
        :param speed: Velocity scaled from -1 (right) to 1 (left)
        """
        for motor in self.lift_motors:
            motor.goal_velocity = speed * motor.speed

    def stop_lift(self):
        """
        Set elevator motors to stop
        """
        for motor in self.lift_motors:
            motor.set_torque = 0

    def strafe_drive(self, x, y):
        """
        Omni-direction driving
        :param x: left right velocity scaled from -1 (right) to 1 (left)
        :param y: forward back velocity scaled from -1 (back) to 1 (forward) 
        """
        v = np.sqrt(x**2 + y**2)
        angle = np.degrees(np.arctan2(x, y))
        if abs(angle) > 135:
            angle -= 180 * np.sign(angle)
            v *= -1
        for motor in self.steer_motors:
            motor.set_angle = angle

        if self.torque_mode:
            print("torque mode strafe drive")
            for motor in self.drive_motors:
                motor.set_torque = v * motor.stall / 2
        else:
            print("vel mode strafe drive")
            for motor in self.drive_motors:
                motor.goal_velocity = v * motor.speed

    def print_lift(self):
        """
        Printing elevator motor positions
        """
        for id in self.lift_ids:
            print(id, self.motors.get(id))

    def turn_torque(self, speed):
        """
        Turn CW/CCW
        :param speed: Angular velocity scaled from 1 (CW) to 1 (CCW)
        """
        drive_dirs = (-1, 1, -1, 1)
        for motor, direction in zip(self.drive_motors, drive_dirs):
            motor.set_torque = direction * speed * motor.stall
        steer_dirs = (1, -1, -1, 1)
        for motor, direction in zip(self.steer_motors, steer_dirs):
            motor.set_angle = direction * 60

    def turn_vel(self, speed):
        """
        Turn CW/CCW
        :param speed: Angular velocity scaled from 1 (CW) to 1 (CCW)
        """
        drive_dirs = (-1, 1, -1, 1)
        for motor, direction in zip(self.drive_motors, drive_dirs):
            motor.goal_velocity = direction * speed * motor.speed
        steer_dirs = (1, -1, -1, 1)
        for motor, direction in zip(self.steer_motors, steer_dirs):
            motor.set_angle = direction * 60

    def turn(self, speed):
        """
        Turn CW/CCW in the current operating mode (velocity or torque)
        :param speed: Angular velocity or torque scaled from 1 (CW) to 1 (CCW)
        """
        if self.torque_mode:
            self.turn_torque(speed)
        else:
            self.turn_vel(speed)

    def stop(self):
        """
        Stop all motors
        """
        for motor in self.drive_motors:
            motor.set_torque = 0
            motor.set_velocity = 0
            motor.goal_velocity = 0

        for motor in self.steer_motors:
            motor.set_velocity = 0
            motor.goal_velocity = 0
            
        for motor in self.lift_motors:
            motor.set_angle = motor.angle
    # CHECK IF THESE ARE USED, Not Checked yet
    def set_motor_torque(self, id, tau):
        """
        Set motor torques to input torque value
        :param id: motor id from 1 to 10
        :param tau: motor torque goal value
        """
        self.motors.get(id).set_torque = tau

    def set_motor_velocity(self, id, v):
        """
        Set motor velociteis to input velocity value
        :param id: motor id from 1 to 10
        :param v: motor velocity goal value
        """
        self.motors.get(id).goal_velocity = v

    def get_motor_torque(self, id):
        """
        Obtain motor torques according to motor id
        :param id: motor id from 1 to 10
        """
        return self.torques[id - 1][-1]
    # END OF CHECK
    
    def zero_elevator(self):
        """
        Set elevator height to ground level
        """
        self.lift_motors[0].set_angle = self.lift_zero
        self.lift_motors[1].set_angle = self.lift_zero

    def raise_elevator(self):
        """
        Set elevator height to maximum
        """
        self.lift_motors[0].set_angle = self.lift_raised
        self.lift_motors[1].set_angle = self.lift_raised

    def lower_elevator(self):
        """
        Set elevator height to minimum (floor)
        """
        self.lift_motors[0].set_angle = self.lift_lowered
        self.lift_motors[1].set_angle = self.lift_lowered

    def update_state(self, orientation):
        """
        Log robot's motor torques, velocities, and orientations
        :param orientation: orientation readings from IMU
        """
        for i, motor in enumerate(self.motors.get()):
            self.torques[i].pop()
            self.torques[i].insert(0, motor.torque)
            self.velocities[i].pop()
            self.velocities[i].insert(0, motor.velocity)
        for i, angle in enumerate(self.orientation):
            angle.pop()
            angle.insert(0, orientation[i])

    def plot_torque_vel(self):
        """
        Plotting each motors' torque and velocity graph
        """
        fig, axs = plt.subplots(self.n, 2)
        fig.suptitle("Vertically stacked subplots")
        for i in range(self.n):
            t = self.time
            torque = self.torques[i]
            velocity = self.velocities[i]
            label_torque = "Motor " + str(i) + " torque"
            label_velocity = "Motor " + str(i) + " velocity"
            axs[i, 0].plot(t, torque, label=label_torque)
            axs[i, 1].plot(t, velocity, label=label_velocity)
        plt.legend()
        plt.show()

    def update_imu(self, acceleration):
        """
        Log robot's acceleration readings
        :params acceleration: acceleration readings from IMU (m/s^2)
        """
        self.acceleration = acceleration

    def get_steer_torques(self):
        """
        From motors.py, read_torques. Values are averaged, in N*mm. See class Motor
        For steering motors
        """
        torque_steer = np.zeros(len(self.steer_motors))
        for i, motor in enumerate(self.steer_motors):
            torque_steer[i] = motor.torque
        return torque_steer

    def get_drive_torques(self):
        """
        From motors.py, read_torques. Values are averaged, in N*mm. See class Motor
        For driving motors
        """
        torque_drive = np.zeros(len(self.drive_motors))
        for i, motor in enumerate(self.drive_motors):
            torque_drive[i] = motor.torque
        return torque_drive

    def get_steer_angles(self):
        """
        From motors.py, read torque angles. Values are averaged, in N*mm. See class Motor
        For driving motors
        """
        theta = np.zeros(len(self.steer_motors))
        for i, motor in enumerate(self.steer_motors):
            theta[i] = motor.angle

        return theta

    def get_hand_jacobian(self):
        """
        Calculates the Jacobian from the configuration of the robot
        """
        vector_r = np.array([[-self.radius], [0], [0]])
        vector_0 = np.zeros([3, 1])
        jh = np.block(
            [
                [vector_r, vector_0, vector_0, vector_0],
                [vector_0, vector_r, vector_0, vector_0],
                [vector_0, vector_0, vector_r, vector_0],
                [vector_0, vector_0, vector_0, vector_r],
            ]
        )
        return jh

    def get_grasp_map(self):
        """
        Calculates the grasp map from the configuration of the robot
        """
        steer_theta = self.get_steer_angles()

        grasp_transpose = np.zeros((3 * len(self.steer_motors), 6))

        r = np.array(
            [
                [self.length / 2, self.length / 2, -self.length / 2, -self.length / 2],
                [self.width / 2, -self.width / 2, self.width / 2, -self.width / 2],
                [-self.height, -self.height, -self.height, -self.height],
            ]
        )

        for i, theta in enumerate(steer_theta):
            rotation = np.array(
                [
                    [np.cos(theta), -np.sin(theta), 0],
                    [np.sin(theta), np.cos(theta), 0],
                    [0, 0, 1],
                ]
            )
            grasp_transpose[i * 3 : i * 3 + 3, :] = np.hstack((rotation, -skew(r[:, i])))
        # This returns G, Gt is transposed back to G
        return grasp_transpose.transpose()

    def get_contact_forces(self):
        """
        Finding contact forces with Jb, G_T, u (actuator joint torques from motors 5 to 8),
        f_ext (external force, gravity in this case)
        """
        # Find u
        u = np.zeros(len(self.drive_ids))
        drive_torques = self.get_drive_torques()
        for i in range(len(self.drive_ids)):
            u[i] = drive_torques[i] / 1000

        # Find f_ext, extract IMU's acceleration data and mul. by m of robot
        f_ext = np.array(
            [
                self.acceleration[0] * self.mass,
                self.acceleration[1] * self.mass,
                self.acceleration[2] * self.mass,
                0,
                0,
                0,
            ]
        )
        # Find N, using 0 as placeholder for simplified mass
        nonlinear = np.zeros(len(self.drive_ids))

        # Find hand Jacobian and grasp map matrices
        jacobian = self.get_hand_jacobian()
        grasp_map = self.get_grasp_map()
        # A = [-Jh';-G]
        a = np.vstack((jacobian.transpose(), grasp_map))
        # b = [u-N, F_ext]
        b = np.hstack((u - nonlinear, -f_ext))

        # using rcond to enforce 3-dimensional null space
        fc = np.linalg.lstsq(a, b, rcond=0.01)[0]

        return fc

    def get_optimized_forces(self):
        """
        Finding optimized forces using solve_qp
        """
        # Find u
        u = np.zeros(len(self.drive_ids))
        drive_torques = self.get_drive_torques()
        for i in range(len(self.drive_ids)):
            u[i] = drive_torques[i] / 1000

        # Find f_ext (external forces), in this case assume gravity only
        f_ext = np.array(
            [
                self.acceleration[0] * self.mass,
                self.acceleration[1] * self.mass,
                self.acceleration[2] * self.mass,
                0,
                0,
                0,  # Assuming no ext. torques
            ]
        )

        # Find N, torques due to gravity (assume zero)
        nonlinear = np.zeros(len(self.drive_ids))

        # Find hand Jacobian and grasp map
        jacobian = self.get_hand_jacobian()
        grasp_map = self.get_grasp_map()

        # Regularization matrix H
        num_forces = jacobian.shape[0]  # Number of contact force variables
        num_vars = num_forces + 1  # Including the adhesion margin 'c'

        h = self.regularization * np.eye(num_vars)

        # Linear cost vector f to max c
        f = np.zeros(num_vars)
        f[-1] = -1  # last term is -1 to maximize c

        # Null space
        null_sp = null_space(np.vstack((-jacobian.T, grasp_map)), rcond=0.01)

        # Equality constraints: Aeq x = beq
        a_eq = np.vstack((grasp_map, null_sp.T))
        a_eq = np.hstack(
            (a_eq, np.zeros((a_eq.shape[0], 1)))
        )  # Add zero column for 'c'

        b_eq = np.hstack(
            (-f_ext, np.zeros(null_sp.shape[1]))
        )  # Static equilibrium constraints

        # Inequality constraints: A x <= b
        a_tor_lower = np.hstack((-jacobian.T, np.zeros((jacobian.T.shape[0], 1))))
        b_tor_lower = nonlinear + self.max_torque  # Torque lower bounds

        a_tor_upper = np.hstack((jacobian.T, np.zeros((jacobian.T.shape[0], 1))))
        b_tor_upper = self.max_torque - nonlinear  # Torque upper bounds

        # Excluding the adhesion margin inequalities
        a_c = np.array(
            [
                [-1, 0, -self.friction],
                [1, 0, -self.friction],
                [0, -1, -self.friction],
                [0, 1, -self.friction],
                [0, 0, -1],
            ]
        )
        b_c = np.array(
            [
                [self.friction * self.magnet_force],
                [self.friction * self.magnet_force],
                [self.friction * self.magnet_force],
                [self.friction * self.magnet_force],
                [self.magnet_force],
            ]
        )

        num_wheels = 4
        a_row, a_col = a_c.shape

        # big 0 matrix
        a_adhesion = np.zeros((num_wheels * a_row, num_wheels * a_col + 1))
        b_adhesion = np.zeros((num_wheels * a_row, 1))

        for i in range(num_wheels):
            row_start = i * a_row
            row_end = (i + 1) * a_row
            col_start = i * a_col
            col_end = (i + 1) * a_col
            # A_c into A_adhesion
            a_adhesion[row_start:row_end, col_start:col_end] = a_c
            # last column of 1 into A_adhesion
            a_adhesion[row_start:row_end, -1] = 1
            # b_c into b_adhesion
            b_adhesion[row_start:row_end, 0] = b_c.flatten()

        b_adhesion = b_adhesion.T
        b_tor_lower = b_tor_lower.reshape(1, -1)  # Reshape to (1, 20)
        b_tor_upper = b_tor_upper.reshape(1, -1)  # Reshape to (1, 20)
        # Stack A and b with adhesion in priority
        a = np.vstack((a_adhesion, a_tor_lower, a_tor_upper))
        b = np.hstack((b_adhesion, b_tor_lower, b_tor_upper))

        x = solve_qp(h, f, a, b, a_eq, b_eq, solver="quadprog")
        f_opt = x[:-1]  # optimized forces
        c = x[-1]  # adhesion margin
        return f_opt, c

    def force_control(self, f_c, f_opt, dt):
        """
        This function takes in the contact forces and optimized forces,
        then calculates the amount of error and outputs the goal velocity
        based on a proportional gain.
        :params f_c: estimated contact force (Nm)
        :params f_opt: optimized goal force (Nm)
        :params dt: time interval of system (s)
        """
        for motor in self.drive_motors:
            motor.set_velocity = motor.goal_velocity

        if not self.force_control_on:
            self.diff_prev = 0
            return

        # For proportional term
        diff_forces = f_opt - f_c
        diff_forces = np.reshape(diff_forces, (len(self.drive_ids) * 3, 1))

        # For derivative term
        df_dt = (diff_forces - self.diff_prev) / dt

        jacobian = self.get_hand_jacobian()  # Use J to convert forces into torques
        dv = self.kp * jacobian.T @ diff_forces + self.kd * jacobian.T @ df_dt
        print("delta v:", dv)
        for i, motor in enumerate(self.drive_motors):
            motor.set_velocity += dv[i]

        self.diff_prev = diff_forces


def skew(vector):
    """
    6*1 velocity vector to 3*3 skew matrix
    """
    return np.array(
        [
            [0, -vector[2], vector[1]],
            [vector[2], 0, -vector[0]],
            [-vector[1], vector[0], 0],
        ]
    )


if __name__ == "__main__":
    #Testing code with fake param.
    from motors import Motors

    robot = Robot(Motors())
    robot.steer_motors[0].angle = 0
    robot.steer_motors[1].angle = 0
    robot.steer_motors[2].angle = 0
    robot.steer_motors[3].angle = 0

    J = robot.get_hand_jacobian()
    G = robot.get_grasp_map()

    print(f"J: \n{J}")
    print(f"G: \n{G}")

    robot.update_imu([-9.8, 0, 0])
    fcontact = robot.get_contact_forces()
    foptimized, cost = robot.get_optimized_forces()

    fcontact = np.array(fcontact).reshape(4, 3)
    print(f"fc: \n{fcontact}")
    foptimized = foptimized.reshape(4, 3)
    print(f"fopt: \n{foptimized}")
