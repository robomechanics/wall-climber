import numpy as np
from math import *
from scipy.optimize import linprog

PI = 3.14159

ALIGN_TORQUE = 0.2
mu = 0.8
g = [0, -9.81]
v_0 = 0.5
kp = 1

# Wheel contact state - can be NONE, FRONT, MID, or REAR 
contact_state = 'NONE'

# State of the transition - can be TELEOP, ALIGN, BREAK_FRONT, CLIMB, BREAK_REAR, or COMPLETE
state = 'TELEOP'   

start_angle = 0
transition_angle = PI/2

# Calculate vector from rear wheel contact to robot CoM
def get_com_vector(robot):
    theta = abs(radians(robot.get_pitch()) - start_angle) # Current robot angle relative to starting surface
    alpha = atan(2 * robot.get_h() / robot.get_l()) # Angle between CoM vector and robot pitch
    angle_vector = np.matrix([cos(alpha + theta)], [sin(alpha + theta)]) # Direction of vector from rear wheel center to CoM
    wheel_offset = np.matrix([0], [robot.get_r()]) # Offset for wheel radius
    r_com = sqrt(robot.get_h() ** 2 + (robot.get_l() / 2) ** 2) * angle_vector + wheel_offset
    return r_com

# Calculate vector from rear wheel contact to front wheel contact
def get_contact_vector(robot):
    theta = abs(radians(robot.get_pitch()) - start_angle) # Current robot angle relative to starting surface
    r_1 = robot.get_l() * np.matrix([cos(theta)], [sin(theta)])
    return r_1

# Get rotation matrix from world frame to rear contact frame
def get_gravity_rot_mat():
    R = np.matrix([cos(start_angle), -sin(start_angle)], [sin(start_angle), cos(start_angle)])
    return R

# Get rotation matrix from front contact frame to rear contact frame
def get_contact_rot_mat():
    R = np.matrix([cos(transition_angle), -sin(transition_angle)], [sin(transition_angle), cos(transition_angle)])
    return R

'''
Solve linprog for wheel torques during climbing phase of transition

Find    [T1, T2]
For     max(s)
s.t.    N_f >= s
        N_r >= s
        |T_f| <= mu(N_f - s)
        |T_r| <= mu(N_r - s)
        0 = R_g*mg + R_f*(N_f + T_f + M_f) + (N_r + T_r + M_r)
        0 = r_1 x (M_f + T_f + N_f) + r_com x R_g*mg

N - wheel normal force
M - wheel magnetic force
T - wheel tangential force
mu - friction coefficient between wheel and surface
mg - gravity force vector
r_1 - vector from rear contact to front contact
r_com - vector from rear contact to robot CoM
R_g - rotation matrix from world frame to rear contact frame
R_f - rotation matrix from front contact frame to rear contact frame

Variable order in solver: [s, T_f_p, T_f_n, T_r_p, T_r_n, N_f, N_r]

To account for absolute value constraint, split the tangential forces into two new variables (T_p and T_n). See https://stackoverflow.com/a/28933583 for
more details.
'''
def solve_climb(robot):

    g_rot = np.matmul(get_gravity_rot_mat(), g)
    M = robot.get_M()
    m = robot.get_mass()
    l = robot.get_l()
    theta = abs(radians(robot.get_pitch()) - start_angle)
    beta = transition_angle - theta

    c = [-1, 0, 0, 0, 0, 0, 0] # min(-s)
    A_ub = [[1,0,0,0,0,-1,0], [1,0,0,0,0,0,-1], [mu,1,1,0,0,-1,0], [mu,0,0,1,1,0,0,-1]]
    b_ub = [0,0,0,0]
    A_eq = [[0,sin(transition_angle),-sin(transition_angle),0,0,cos(transition_angle),1],
            [0,cos(transition_angle),-cos(transition_angle),1,-1,-sin(transition_angle),0],
            [0,sin(beta),-sin(beta),0,0,cos(beta),0]]
    b_eq = [M*(1+cos(transition_angle)) + m * g_rot[1],
            m * g_rot[0] - M*sin(theta),
            -m/l * np.cross(get_com_vector(), g_rot) + M*cos(beta)]
    bounds = [(0, None), (0, None), (0, None), (0, None), (0, None), (None, None), (None, None)]

    res = linprog(c, A_ub=A_ub, b_ub=b_ub, A_eq=A_eq, b_eq=b_eq, bounds=bounds)

    if res.success:
        return [robot.get_r()*(res.x[1] - res.x[2]), robot.get_r()*(res.x[3] - res.x[4])] # Desired wheel torques
    else:
        return None

def solve_break_front(robot):
    return

def loop(robot):
    if state == 'TELEOP':
        contact_state = 'NONE'
        start_angle = radians(robot.get_pitch())
    elif state == 'ALIGN':
        for id in [5,6]:
            robot.set_torque_mode(id)
        for id in [7,8]:
            robot.set_velocity_mode(id)
        if robot.get_motor_velocity(5) == 0 & robot.get_motor_velocity(6) == 0:
            robot.stop()
            state = 'COMPLETE'
            # state = 'CLIMB'
        else:
            if robot.get_motor_velocity(5) != 0:
                robot.set_motor_torque(5, ALIGN_TORQUE)
                robot.set_motor_velocity(7, robot.get_motor_velocity(5))
            if robot.get_motor_velocity(6) != 0:
                robot.set_motor_torque(6, ALIGN_TORQUE)
                robot.set_motor_velocity(8, robot.get_motor_velocity(6))
    elif state == 'BREAK_FRONT':
        pass
    elif state == 'CLIMB':
        if transition_angle - abs(radians(robot.get_pitch()) - start_angle) > radians(2):
            [t_f, t_r] = solve_climb(robot)
            v_fs = [0,0]
            v_rs = [0,0]
            for i in range(2):
                v_fs[i] = v_0 + kp * (t_f - robot.get_motor_velocity(i+4))
            for i in range(2):
                v_rs[i] = v_0 + kp * (t_r - robot.get_motor_velocity(i+6))
        else:
            robot.stop()
            state == 'BREAK_REAR'
    elif state == 'BREAK_REAR':
        pass
    elif state == 'COMPLETE':
        robot.mode = 0
        state = 'TELEOP'