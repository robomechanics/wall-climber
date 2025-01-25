"""
Evaluate current robot state and modify setpoints using forward and inverse kinematics
"""

import numpy as np
import matplotlib.pyplot as plt
from qpsolvers import solve_qp

# Wheel numbering
FL = 1  # Front left
FR = 2  # Front right
RL = 3  # Rear left
RR = 4  # Rear right

drive_ids = (5, 6, 7, 8)
steer_ids = (1, 2, 3, 4)
lift_ids = (9, 10)
steer_offsets = (0, -20, 45, 0)

elevator_left_offset = -253
elevator_right_offset = -406

lamb = 1e-6
# Lower and upper bounds of torque for optimization
lb = 0
ub = 10

class Robot:
    def __init__(
        self, motors, l=0.433, w=0.317, h=0.064, r=0.025, M=50, mass=8.3
    ):  # default parameters are Sally's
        self.drive_motors = motors.add(drive_ids, "XM430-W210-T", mirror=(5, 7))
        self.steer_motors = motors.add(
            steer_ids,
            "XM430-W210-T",
            offset={steer_ids[i]: steer_offsets[i] for i in range(4)},
        )
        self.lift_motors = motors.add(lift_ids, "XM430-W210-T", mirror=(9,))

        motors.enable(drive_ids, torque_mode=True)
        motors.enable(steer_ids, velocity_mode=False)
        motors.enable(lift_ids, velocity_mode=False)

        self.drive_ids = drive_ids
        self.motors = motors

        self.l = l
        self.w = w
        self.h = h
        self.r = r
        self.M = M

        self.mass = mass
        self.acc = [0, 0, 0]

        self.time = []
        self.torques = [[0 for i in range(10)] for j in range(10)]
        self.velocities = [[0 for i in range(10)] for j in range(10)]
        self.orientation = [[0 for i in range(10)] for j in range(3)]

        # For optimization
        self.lamb = lamb
        self.lb = lb
        self.ub = ub
        # 0 is teleop, 1 is transition
        self.mode = 0

        for i, id in enumerate(lift_ids):
            self.motors.get(id).goal_torque = 400
            self.motors.get(id).set_torque = 400
        for i, id in enumerate(drive_ids):
            self.motors.get(id).goal_torque = 0
            self.motors.get(id).set_torque = 0

    def get_l(self):
        return self.l

    def get_h(self):
        return self.h

    def get_r(self):
        return self.r

    def get_M(self):
        return self.M

    def get_mass(self):
        return self.mass

    def get_pitch(self):
        return sum(self.orientation[0]) / len(self.orientation[0])

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
        v = np.sqrt(x**2 + y**2)
        angle = np.degrees(np.arctan2(x, y))
        if abs(angle) > 135:
            angle -= 180 * np.sign(angle)
            v *= -1
        for i, id in enumerate(steer_ids):
            self.motors.get(id).set_angle = angle

        # for i, id in enumerate(drive_ids):
        #    self.motors.get(id).set_velocity = v * self.motors.get(id).speed
        if self.motors.get(7).torque_mode:
            print("torque mode strafe drive")
            for i, id in enumerate(drive_ids):
                self.motors.get(id).set_torque = v * self.motors.get(id).stall / 2
        if self.motors.get(7).velocity_mode:
            print("vel mode strafe drive")
            for i, id in enumerate(drive_ids):
                self.motors.get(id).set_velocity = v * self.motors.get(id).speed

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
            self.motors.get(id).set_torque = (
                drive_dirs[i] * v * self.motors.get(id).stall / 1
            )
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
            self.motors.get(id).set_velocity = (
                drive_dirs[i] * v * self.motors.get(id).speed
            )
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

    def set_torque_mode(self, id):
        self.motors.get(id).torque_mode = True
        self.motors.get(id).velocity_mode = False

    def set_velocity_mode(self, id):
        self.motors.get(id).torque_mode = False
        self.motors.get(id).velocity_mode = True

    def set_motor_torque(self, id, t):
        self.motors.get(id).set_torque = t

    def set_motor_velocity(self, id, v):
        self.motors.get(id).set_velocity = v

    def get_motor_velocity(self, id):
        return self.velocities[id - 1][-1]

    def get_motor_torque(self, id):
        return self.torques[id - 1][-1]

    def zero_elevator(self):
        self.lift_motors[0].set_angle = 13.4 + elevator_left_offset + 100
        self.lift_motors[1].set_angle = 14.1 + elevator_right_offset + 100

    def raise_elevator(self):
        self.lift_motors[0].set_angle = -613.3 + elevator_left_offset + 250
        self.lift_motors[1].set_angle = -590.1 + elevator_right_offset + 250

    def lower_elevator(self):
        self.lift_motors[0].set_angle = 194.8 + elevator_left_offset + 100
        self.lift_motors[1].set_angle = 156.4 + elevator_right_offset + 100

    def update_state(self, orientation):
        motor_ids = range(1, 11)
        for id in motor_ids:
            self.torques[id - 1].pop()
            self.torques[id - 1].insert(0, self.motors.get(id).torque)
            self.velocities[id - 1].pop()
            self.velocities[id - 1].insert(0, self.motors.get(id).velocity)
        for i in range(len(self.orientation)):
            self.orientation[i].pop()
            self.orientation[i].insert(0, orientation[i])
        print(
            f"Orientation: {orientation[0]:.3f}, {orientation[1]:.3f}, {orientation[2]:.3f}"
        )

    def get_motor_info(self):
        motor_ids = range(1, 11)
        self.motors.read_torque(ids=motor_ids)
        self.motors.read_velocity(ids=motor_ids)
        for id in motor_ids:
            self.torques[id - 1].append(self.motors.get(id).torque)
            self.velocities[id - 1].append(self.motors.get(id).velocity)

    def plot_torque_vel(self):
        fig, axs = plt.subplots(10, 2)
        fig.suptitle("Vertically stacked subplots")
        for i in range(10):
            t = self.time
            torque = self.torques[i]
            velocity = self.velocities[i]
            label_torque = "Motor " + str(i) + " torque"
            label_velocity = "Motor " + str(i) + " velocity"
            axs[i, 0].plot(t, torque, label=label_torque)
            axs[i, 1].plot(t, velocity, label=label_velocity)
        plt.legend()
        plt.show()

    #######################################################################################
    # Some functions for force estimation
    def update_imu(self, acc):
        self.acc = acc
        # print(f"Acc from update_imu: \n{acc[0]:.3f}, {acc[1]:.3f}, {acc[2]:.3f}")

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

        theta = np.zeros(len(self.steer_motors))
        for i, motor in enumerate(self.steer_motors):
            theta[i] = motor.angle

        return theta

    def get_hand_Jacobian(self):

        vector_r = np.array([[-self.r], [0], [0]])
        vector_0 = np.zeros([3, 1])
        Jh = np.block(
            [
                [vector_r, vector_0, vector_0, vector_0],
                [vector_0, vector_r, vector_0, vector_0],
                [vector_0, vector_0, vector_r, vector_0],
                [vector_0, vector_0, vector_0, vector_r],
            ]
        )
        return Jh

    def get_grasp_map(self):

        steer_theta = self.get_steer_angles()

        G_T = np.zeros((3 * len(self.steer_motors), 6))

        r = np.array(
            [
                [self.l / 2, self.l / 2, -self.l / 2, -self.l / 2],
                [self.w / 2, -self.w / 2, self.w / 2, -self.w / 2],
                [-self.h, -self.h, -self.h, -self.h],
            ]
        )

        for i, theta in enumerate(steer_theta):
            R = np.array(
                [
                    [np.cos(theta), -np.sin(theta), 0],
                    [np.sin(theta), np.cos(theta), 0],
                    [0, 0, 1],
                ]
            )
            G_T[i * 3 : i * 3 + 3, :] = np.hstack((R, -skew(r[:, i])))
        # This returns G, Gt is transposed back to G
        return G_T.transpose()

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
                self.acc[0] * self.mass,
                self.acc[1] * self.mass,
                self.acc[2] * self.mass,
                0,
                0,
                0,
            ]
        )

        # Find N, using 0 as placeholder for simplified mass
        N = np.zeros(len(self.drive_ids))

        # b = [u-N; F_ext]
        b = np.hstack((u - N, -f_ext))

        # Find hand Jacobian and grasp map matrices
        J = self.get_hand_Jacobian()
        G = self.get_grasp_map()

        # A = [-Jh';-G]
        A = np.vstack((J.transpose(), G))

        # Contact force
        # fc = np.linalg.lstsq(A, b, rcond=None)[0]

        # Using regularization to penalize sensor noise
        lambda_reg = 0.0001
        A_reg = A.T @ A + np.eye(A.shape[1]) * lambda_reg
        b_reg = A.T @ b
        fc = np.linalg.lstsq(A_reg, b_reg, rcond=None)[0]
        # print(f"Acc: \n{self.acc[2]}")
        # print(f"Contact Forces: \n{fc[0:3]},\n{fc[3:6]},\n{fc[6:9]},\n{fc[9:12]}\n")
        print(A)
        print(b)
        print(fc)
        return fc
    
    def get_optimized_forces(self):
        # Find u
        u = np.zeros(len(self.drive_ids))
        drive_torques = self.get_drive_torques()
        for i in range(len(self.drive_ids)):
            u[i] = drive_torques[i] / 1000

        # Find f_ext (external forces), in this case assume gravity only
        f_ext = np.array([
            self.acc[0] * self.mass,
            self.acc[1] * self.mass,
            self.acc[2] * self.mass,
            0, 0, 0  # Assuming no ext. torques
        ])

        # Find N (assume zero)
        N = np.zeros(len(self.drive_ids))

        # Find hand Jacobian and grasp map
        J = self.get_hand_Jacobian()
        G = self.get_grasp_map()

        # Regularization matrix H
        num_forces = J.shape[0]  # Number of contact force variables
        num_vars = num_forces + 1  # Including the adhesion margin 'c'

        H = self.lamb * np.eye(num_vars)

        # Linear cost vector f to max c
        f = np.zeros(num_vars)
        f[-1] = -1  # last term is -1 to maximize c

        # Equality constraints: Aeq x = beq
        A_eq = np.hstack((G, np.zeros((G.shape[0], 1))))  # Add zero column for 'c'
        b_eq = -f_ext  # Static equilibrium constraints

        # Inequality constraints: A x <= b
        A_adhesion = np.hstack((J.T, np.ones((J.shape[1], 1))))  # Adhesion constraints
        b_adhesion = np.zeros(J.shape[1])  # Adjust based on adhesion limits

        A_tor_lower = np.hstack((-J.T, np.zeros((J.T.shape[0], 1))))
        b_tor_lower = self.lb - N  # Torque lower bounds

        A_tor_upper = np.hstack((J.T, np.zeros((J.T.shape[0], 1))))
        b_tor_upper = self.ub - u  # Torque upper bounds

        # Stack A and b with adhesion in priority
        A = np.vstack((A_adhesion, A_tor_lower, A_tor_upper))
        b = np.hstack((b_adhesion, b_tor_lower, b_tor_upper))

        # Formatting for qp solvers
        lb = np.full(num_vars, -np.inf)
        ub = np.full(num_vars, np.inf)
        lb[:num_forces] = self.lb
        ub[:num_forces] = self.ub
        lb[-1] = 0

        # qp solver to find maximum
        x = solve_qp(H, f, A, b, A_eq, b_eq, lb, ub, solver="quadprog")

        f_opt = x[:-1] # optimized forces
        c = x[-1] # adhesion margin
        return f_opt, c

def skew(vector):
    # 6*1 velocity vector to 3*3 skew matrix
    return np.array(
        [
            [0, -vector[2], vector[1]],
            [vector[2], 0, -vector[0]],
            [-vector[1], vector[0], 0],
        ]
    )


if __name__ == "__main__":
    from motors import Motors

    robot = Robot(Motors(), l=10, w=6, h=1, r=1)  # Testing with fake param.
    robot.steer_motors[0].angle = 0
    robot.steer_motors[1].angle = 0
    robot.steer_motors[2].angle = 0
    robot.steer_motors[3].angle = 0

    J = robot.get_hand_Jacobian()
    G = robot.get_grasp_map()

    print(f"J: \n{J}")
    print(f"G: \n{G}")

    robot.update_imu([0, 0, -9.8])
    fc = robot.get_contact_forces()
    print(f"fc: \n{fc}")
