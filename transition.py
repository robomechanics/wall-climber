import numpy as np
from math import *
from scipy.optimize import linprog

PI = 3.14159

ALIGN_TORQUE = 0.2

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
    angle_vector = np.array([cos(transition_angle + theta)], [sin(transition_angle + theta)]) # Direction of vector from rear wheel center to CoM
    wheel_offset = np.array([0], [robot.get_r()]) # Offset for wheel radius
    r_com = sqrt(robot.get_h() ** 2 + (robot.get_l() / 2) ** 2) * angle_vector + wheel_offset
    return r_com

# Calculate vector from rear wheel contact to front wheel contact
def get_contact_vector(robot):
    theta = abs(radians(robot.get_pitch()) - start_angle) # Current robot angle relative to starting surface
    r_1 = robot.get_l() * np.array([cos(theta)], [sin(theta)])
    return r_1

# Get rotation matrix from world frame to rear contact frame
def get_gravity_rot_mat():
    R = np.array([cos(start_angle), -sin(start_angle)], [sin(start_angle), cos(start_angle)])
    return R

# Get rotation matrix from front contact frame to rear contact frame
def get_contact_rot_mat():
    R = np.array([cos(transition_angle), sin(transition_angle)], [sin(transition_angle), cos(transition_angle)])
    return R

'''
Solve linprog for wheel torques required to break front wheel contact with initial surface

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
'''
def solve_break_front(robot):
    return

def solve_climb(robot, pitch):
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
            state = 'BREAK_FRONT'
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
        pass
    elif state == 'BREAK_REAR':
        pass
    elif state == 'COMPLETE':
        robot.mode = 0
        state = 'TELEOP'