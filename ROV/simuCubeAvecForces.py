import sys
import os

parent_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
sys.path.append(parent_dir)

from roblib import *
from manette import *

# Constants for the BlueROV (using Fossen notations)
def initialize_parameters():
    # Mass, inertia, damping, added mass, thruster configuration
    m = 11.5  # Mass of the BlueROV
    I = np.diag([0.1, 0.1, 0.2])  # Inertia matrix
    D = np.diag([0.5, 0.5, 0.5])  # Damping coefficients
    Ma = np.diag([1.5, 1.5, 1.5]) # Added mass terms

    # Thruster configuration
    T = np.array([[1, 1, 1, 1],  # Propeller configuration matrix
                  [0, 0, 0, 0],
                  [-0.1, 0.1, -0.1, 0.1]])
    
    return m, I, D, Ma, T


def equations_of_motion(x, u, m, I, D, Ma, T):
    # State vector: [x, y, z, phi, theta, psi, u, v, w, p, q, r]
    # Control input u: thrust forces from propellers
    v = x[6:9]  # Linear velocities
    omega = x[9:12]  # Angular velocities

    # Hydrodynamic forces and moments
    F_hydro = -D @ v  # Linear drag
    M_hydro = -D @ omega  # Angular drag

    # Thruster forces
    F_thruster = T @ u[:4]  # Apply thrust forces to body
    M_thruster = T @ u[4:]  # Apply moment from thrusters

    # Total forces and moments
    F_total = F_thruster + F_hydro
    M_total = M_thruster + M_hydro

    # Equations of motion (Fossen 3DOF model)
    dv = (1/m) * (F_total - np.cross(omega, m*v))
    domega = np.linalg.inv(I) @ (M_total - np.cross(omega, I @ omega))

    # Return derivatives
    return np.hstack((v, omega, dv, domega))


def control(x, target_velocity, target_omega, m, D, T):
    # Linear velocity control (example with PD controller)
    v = x[6:9]  # Current velocity
    omega = x[9:12]  # Current angular velocity

    # Error in velocities
    v_error = target_velocity - v
    omega_error = target_omega - omega

    # Thruster control signals (simplified)
    u_thrust = np.linalg.pinv(T) @ (D @ v_error)
    u_moment = np.linalg.pinv(T) @ (D @ omega_error)

    return np.hstack((u_thrust, u_moment))



def update_command(key, target_velocity, target_omega):
    if key == 'up':
        target_velocity[0] += 1  # Increase forward speed
    elif key == 'down':
        target_velocity[0] -= 1  # Decrease forward speed
    if key == 'left':
        target_omega[2] += 0.1  # Rotate to the left
    elif key == 'right':
        target_omega[2] -= 0.1  # Rotate to the right
    if key == 'u':
        target_velocity[2] += 0.1  # Increase upward speed
    elif key == 'd':
        target_velocity[2] -= 0.1  # Decrease upward speed
    return target_velocity, target_omega


def draw_bluerov(ax, x):
    # Drawing the BlueROV in 3D with its current state (position and orientation)
    xr,yr,zr = x[:3]
    position = np.array([[xr],[yr],[zr]])
    
    orientation = eulermat(x[3], x[4], x[5])  # Euler angles to rotation matrix
    # Drawing logic using your favorite 3D plotting library
    print(position)
    print(orientation)
    draw_rov3D(ax,position,orientation)
    #draw_cube3D(ax, position, orientation,size = 10)



if __name__ == "__main__":
    init_pygame()
    
    m, I, D, Ma, T = initialize_parameters()
    
    # Initial conditions
    x = np.zeros(12)  # Initial state [position, orientation, linear and angular velocities]
    target_velocity = np.zeros(3)
    target_omega = np.zeros(3)

    dt = 0.1  # Time step
    running = True
    ax = figure3D()

    while running:
        clean3D(ax, -30, 30, -30, 30, -30, 30)
        draw_bluerov(ax, x)

        running, key = get_values(running)
        target_velocity, target_omega = update_command(key, target_velocity, target_omega)

        u = control(x, target_velocity, target_omega, m, D, T)
        x = x + dt * equations_of_motion(x, u, m, I, D, Ma, T)
        
        pause(0.01)

    quit_pygame(running)



