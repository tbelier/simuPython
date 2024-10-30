from roblib import *
from manette import *
from scipy.integrate import ode

def f(t, x, u):  # fonction qui permet de faire évoluer le système
    x = x.flatten()
    u = u.flatten()
    u0, u1, u2, u3 = list(u)
    phi, theta, psi = list(x[3:6])
    v = x[6]

    E = eulermat(phi, theta, psi)

    # Définition des paramètres utilisés plus tard
    wr = v * B @ np.array([[u1], [u2], [u3]])
    a_x = k1 * u0**2 - k2 * v**2
    dp = E @ array([[v], [0], [0]])
    dv = a_x
    dphidpsi = eulerderivative(phi, theta, psi) @ wr
    return vstack((dp, dphidpsi, dv))

def control(x, ax_bar, wr_bar):
    v = x[6]
    u0 = np.sqrt((ax_bar + k2 * v**2) / k1)
    u123 = inv(B) @ wr_bar / v
    u = vstack((u0, u123))
    return u

def change_order(key, ax_bar, wr_bar):
    ax_bar = 0.1 * (1 - x[6])
    wx, wy, wz = np.array(wr_bar).flatten()
    if str(key) == "left":
        wz = min(wz + 0.1, 0.1)
    if str(key) == "right":
        wz = max(wz - 0.1, -0.1)

    if str(key) == "up":
        wy = min(wy + 0.1, 0.1)
    if str(key) == "down":
        wy = max(wy - 0.1, -0.1)

    return ax_bar, [[0], [wy], [wz]]

if __name__ == "__main__":
    init_pygame()
    running = True

    # Initialisation des variables
    ax = figure3D()
    x = array([[0, 0, -5, 0, 0, 0, 0.1]]).T
    alpha = 0
    u = np.array([[2], [0], [0], [0]])
    dt = 0.1
    k1, k2, L, r = 1, 1, 1, 0.5  # paramètres qui dépendent de la géométrie du robot
    B = array([[-L, -L, -L],
                [0, r * sin(2 * pi / 3), -r * sin(2 * pi / 3)],
                [-r, r * cos(2 * pi / 3), -r * cos(2 * pi / 3)]])
    ax_bar, wr_bar = [[0], [0], [0]], [[0], [0], [0]]

    # Initialisation de l'intégrateur
    solver = ode(lambda t, y: f(t, y, u)).set_integrator('dopri5')
    solver.set_initial_value(x.flatten(), 0)

    # Boucle de l'animation
    while running:
        clean3D(ax, -30, 30, -30, 30, -30, 5)
        draw_riptide(ax, x, u, alpha)

        running, key = get_values(running)
        ax_bar, wr_bar = change_order(key, ax_bar, wr_bar)

        u = control(x, ax_bar, wr_bar)

        # Avancer le temps
        solver.integrate(solver.t + dt)
        x = solver.y.reshape(-1, 1)  # Mettre à jour x avec la solution de l'intégrateur

        alpha = alpha + dt * u[0]

    quit_pygame(running)
