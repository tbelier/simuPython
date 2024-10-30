from simpy_control_ballasts import fct_A_control,fct_b_control,fct_v_control
from roblib import *  # available at https://www.ensta-bretagne.fr/jaulin/roblib.py

ech = 5

a = 1  # length of the cube
L = 2  # longueur entre les centres des 2 ballasts
g = 9.81
Jzz = 10 # Choisi arbitrairement pour compenser L*g bans le calcul de dtheta
rho0 = 1
beta=0.5

ma0,mb0=a**3*rho0/2,a**3*rho0/2
m0=ma0+mb0
Va0,Vb0=a**3,a**3
Veau_max=a**3

def orient_water_ballast(Veau,theta):
    d = a * Veau/Veau_max
    S = d * a
    R = array([[cos(theta), -sin(theta)], [sin(theta), cos(theta)]])
    if 0 < theta < pi / 2:
        cd = a * sin(theta)
        cg = a * cos(theta)
        if cd < cg:
            Smin = tan(theta) * a ** 2 / 2
            if S <= Smin:
                cx = sqrt(2 * S / tan(theta))
                cy = cx * tan(theta)
                P = array([[-a / 2, a / 2], [a / 2, a / 2], [a / 2, -a / 2],
                           [cx - a / 2, -a / 2], [-a / 2, cy - a / 2]]).T
            elif S >= a ** 2 - Smin:
                Sbar = a ** 2 - S
                cx = sqrt(2 * Sbar / tan(theta))
                cy = cx * tan(theta)
                P = array([[a / 2, a / 2], [a / 2, a / 2 - cy], [a / 2 - cx, a / 2]]).T
            else:
                cx = 1 / a * (S - a ** 2 *tan(theta)/ 2)
                cy = cx + a * tan(theta)
                P = array([[-a / 2, a / 2], [a / 2, a / 2], [a / 2, cx - a / 2], [-a / 2, cy - a / 2]]).T
        else:
            Smin = a ** 2 / (2 * tan(theta))
            if S <= Smin:
                cx = sqrt(2 * S / tan(theta))
                cy = cx * tan(theta)
                P = array([[-a / 2, a / 2], [a / 2, a / 2], [a / 2, -a / 2],
                           [cx - a / 2, -a / 2], [-a / 2, cy - a / 2]]).T
            elif S >= a ** 2 - Smin:
                Sbar = a ** 2 - S
                cx = sqrt(2 * Sbar / tan(theta))
                cy = cx * tan(theta)
                P = array([[a / 2, a / 2], [a / 2, a / 2 - cy], [a / 2 - cx, a / 2]]).T
            else:
                cx = 1 / a * (S - a ** 2 /(2*tan(theta)))
                cy = cx + a / tan(theta)
                P = array([[a / 2, a / 2], [a/2,-a/2],[cy-a/2, -a / 2], [cx-a/2, a / 2]]).T
        P=R@P
    elif -pi / 2 < theta < 0.:
        theta=-theta
        cd = a * cos(theta)
        cg = a * sin(theta)
        if cd<cg:
            Smin = a ** 2 / (2 * tan(theta))
            if S <= Smin:
                cx = sqrt(2 * S / tan(theta))
                cy = cx * tan(theta)
                P= array([[-a / 2, -a / 2], [-a / 2, a / 2], [a / 2, a / 2],
                               [a / 2, cy - a / 2], [a / 2-cx, - a / 2]]).T
            elif S>=a**2-Smin:
                Sbar = a ** 2 - S
                cx = sqrt(2 * Sbar / tan(theta))
                cy = cx * tan(theta)
                P = array([[-a / 2, a / 2], [cx - a / 2, a / 2], [-a / 2, a / 2 - cy]]).T
            else:
                cx = 1 / a * (S - a ** 2 / (2 * tan(theta)))
                cy = cx + a / tan(theta)
                P = array([[-a / 2, -a / 2], [-a / 2, a / 2], [a / 2 - cx, a / 2], [a / 2 - cy, -a / 2]]).T
        else:
            Smin = tan(theta) * a ** 2 / 2
            if S <= Smin:
                cx = sqrt(2 * S / tan(theta))
                cy = cx * tan(theta)
                P = array([[-a / 2, -a / 2], [-a / 2, a / 2], [a / 2, a / 2],
                               [a / 2, cy - a / 2], [a / 2-cx, - a / 2]]).T
            elif S>=a**2-Smin:
                Sbar = a ** 2 - S
                cx = sqrt(2 * Sbar / tan(theta))
                cy = cx * tan(theta)
                P = array([[-a / 2, a / 2], [cx - a / 2, a / 2], [-a / 2, a / 2 - cy]]).T
            else:
                cx = 1 / a * (S - a ** 2 *tan(theta)/ 2)
                cy = cx +a*tan(theta)
                P = array([[-a / 2, a / 2],[a / 2, a / 2], [a / 2, cy-a / 2], [-a / 2, cx-a / 2]]).T
        P=R@P
    else:
        P=array([[-a / 2, a / 2], [-a / 2, -a / 2 + d], [a / 2, -a / 2 + d], [a / 2, a / 2]]).T
    return P

def draw_buoy(ax,x):
    clear(ax)
    z, vz, theta, dtheta, Vaeau, Vbeau = x.flatten()
    za = z-L/2*sin(theta)
    zb = z+L/2*sin(theta)
    plot([-10, 10], [0, 0], 'black', linewidth=1)
    P = array([[-ech, -1.8 * ech], [ech, -1.8 * ech], [ech, 0], [-ech, 0]])
    draw_polygon(ax, P, 'blue')

    square=array([[-a/2,a/2],[-a/2,-a/2],[a/2,-a/2],[a/2,a/2],[-a/2,a/2]]).T
    pos_a = array([[-L / 2*cos(theta), za]]).T
    pos_b = array([[L / 2*cos(theta), zb]]).T

    R=array([[cos(theta),-sin(theta)],[sin(theta),cos(theta)]])

    # Draw ballasts
    square=R@square
    square_a=pos_a+square
    square_b=pos_b+square
    servo_link = array([[(-L/2+a/2)*cos(theta), za+a/2*sin(theta)], [(L / 2-a/2)*cos(theta), zb-a/2*sin(theta)]])

    plot(square_a[0,:],square_a[1,:], 'black', linewidth=3)
    plot(square_b[0,:], square_b[1,:], 'black', linewidth=3)
    plot(servo_link[:, 0], servo_link[:, 1], 'black', linewidth=3)

    # Draw air in ballasts
    shape_a=orient_water_ballast(Vaeau,theta)
    shape_b=orient_water_ballast(Vbeau,theta)
    P_air_left = (pos_a + shape_a).T
    draw_polygon(ax, P_air_left, 'white')
    P_air_right = (pos_b + shape_b).T
    draw_polygon(ax, P_air_right, 'white')


def f(x, u):
    z, vz, theta, dtheta, Vaeau, Vbeau = x.flatten()
    ua, ub = u.flatten()
    # Point d'équilibre en z : ba = bb = 0
    return array([[vz],
                  [g*(-1+(Va0+Vb0-(Vaeau+Vbeau))*rho0/m0)],
                  [dtheta],
                  [g*(L/2)*cos(theta)/Jzz*(mb0-ma0+(Va0-Vb0-Vaeau+Vbeau)*rho0)],
                  [ua],
                  [ub]])

def get_depth(x,Gbeta):
    z, vz, theta, dtheta, ba, bb = x.flatten()
    return array([[z]])+mvnrnd1(Gbeta)




def control(x,w,dw,d2w,d3w):
    z, vz, theta, theta_dot, Vaeau, Vbeau = x.flatten()
    A_control=fct_A_control(z,vz,theta,theta_dot,Vaeau,Vbeau,rho0,g,L,Jzz,Va0,Vb0,ma0,mb0)
    b_control = fct_b_control(z,vz,theta,theta_dot,Vaeau,Vbeau,rho0,g,L,Jzz,Va0,Vb0,ma0,mb0)
    v = fct_v_control(z,vz,theta,theta_dot,Vaeau,Vbeau,rho0,g,L,Jzz,Va0,Vb0,ma0,mb0, w, dw, d2w, d3w)
    u = inv(A_control)@(v-b_control)
    return u

def control_PD(x,sum_ez):
    z, vz, theta, theta_dot, VAw, VBw = x.flatten()
    z_des=-4
    theta_des=0
    ez=z_des-z
    sum_ez+=ez
    u3 = - 0.2*ez - 0.5*sum_ez #+ 0.2*vz #- 0.01*(theta_des-theta) + 0.1*theta_dot
    u4 = - 0.2*ez - 0.5*sum_ez #+ 0.2*vz #+ 0.01*(theta_des-theta) - 0.1*theta_dot
    return array([[u3,u4]]).T

def limit_V(x):
    for k in range(2):
        x[4+k]=max(0,min(x[4+k],Veau_max))

def observation(x,xhat):
    z, vz, theta, theta_dot, Vaeau, Vbeau=x.flatten()
    zhat, vzhat=xhat.flatten()
    return array([[zhat,vzhat,theta,theta_dot, Vaeau, Vbeau]]).T


def simu_control():
    x = array([[-5], [0], [pi/6], [0.3], [Veau_max/2], [Veau_max/2]])  # z, vz, theta, dtheta, Vaeau, Vbeau

    # Discrétisation
    tmax, dt = 100, 0.05

    # Etat désiré
    w=array([[-2,0]]).T
    dw=array([[0,0]]).T
    d2w=array([[0,0]]).T
    d3w=array([[0,0]]).T

    # init graph
    ax = init_figure(-ech, ech, -1.8 * ech, 0.2 * ech)

    for t in arange(0, tmax, dt):
        draw_buoy(ax,x)

        u = control(x,w,dw,d2w,d3w)
        x = x + dt * f(x, u)
        limit_V(x)



def simu_kalman():

    x = array([[-2], [0], [0.8], [0], [0*Veau_max/2], [Veau_max]])  # z, vz, theta, dtheta, Vaeau, Vbeau

    # Discrétisation
    tmax, dt = 100, 0.01

    # init graph
    ax = init_figure(-ech, ech, -1.8 * ech, 0.2 * ech)

    sum_ez=0
    for t in arange(0, tmax, dt):


        # Etat désiré
        #w, dw, d2w, d3w = array([[-4+sin(t), 0]]).T, array([[cos(t), 0]]).T, array([[-sin(t), 0]]).T, array([[-cos(t), 0]]).T
        w, dw, d2w, d3w = array([[-6, np.pi/4]]).T, array([[0, 0]]).T, array([[0, 0]]).T, array([[0, 0]]).T

        # Controle

        u = control(x, w, dw, d2w, d3w)

        # Affichage simu
        step=5 # affichage tous les step pas de temps
        if t / dt % step == 0:
            z, vz, theta, dtheta, Vaeau, Vbeau = x.flatten()
            draw_buoy(ax, x)
            draw_arrow(-3, z, pi / 2, vz, col='green', w=2)

        x = x + dt * f(x, u)
        limit_V(x)

    pause(3)

if __name__=="__main__":
    simu_kalman()

