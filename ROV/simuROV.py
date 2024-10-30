import sys
import os

parent_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
sys.path.append(parent_dir)

from roblib import *
from manette import *
from ROV import ROV


def change_order(key,tau_bar):
    Fx_bar,Fy_bar,Fz_bar,Tx_bar,Ty_bar,Tz_bar = tau_bar.flatten()
    
    if str(key) == "left":
        Tz_bar = min(100,Tz_bar+10)
    if str(key) == "right":
        Tz_bar = max(-100,Tz_bar-10)
    if str(key) == "up":
        Ty_bar= min(100,Ty_bar+10)
    if str(key) == "down":
        Ty_bar=max(-100,Ty_bar-10)
    if str(key) == "z":
        Fx_bar= min(100,Fx_bar+10)
    if str(key) == "s":
        Fx_bar=max(-100,Fx_bar-10)
    print("-------- Command Control ---------")
    print(f"Fx = {Fx_bar}")
    return np.array([[Fx_bar],[Fy_bar],[Fz_bar],[Tx_bar],[Ty_bar],[Tz_bar]])

if __name__ == "__main__":
    init_pygame()
    running=True   
    
    # Initialisation des variables
    ax = figure3D()
    eta0 = array([[0,0,-1,0,0,0]]).T
    nu0 = array([[0,0,0,0,0,0]]).T
    nup0 = array([[0,0,0,0,0,0]]).T
    rov = ROV(eta0,nu0,nup0)

    u=np.array([[1500],[1500],[1500],[1500],[1500],[1500],[1500],[1500]]) 
    """
    Commande  donne un [[0.0], [0.0], [0.0], [0.0], [51.502500000000005], [-39.926700000000004], [-39.926700000000004], [51.502500000000005]] et un 
    tau=[[   0.    ]
         [   0.    ]
         [-182.8584]        OU np.array([[0.],[0.],[-182.8584],[0.],[0.],[0.]])
         [   0.    ]
         [   0.    ]
         [   0.    ]]
    """
    dt = 0.01
    tau_bar = np.array([[0],[0],[0],[0],[0],[0]])
    eta_bar, nu_bar = [[0],[0],[0]],[[0],[0],[0]]

    # Boucle de l'animation
    running=True
    k=0
    while running:
        print(f"---- Passage de boucle numéro : {k}")
        k+=1
        running,key=get_values(running)

        tau_bar = change_order(key,tau_bar)
        u =rov.control(tau_bar)
        #etapp = rov.f(u)
        etapp = rov.f(tau_bar)
        rov.integrate(etapp,dt)

        x,y,z,phi,theta,psi = rov.eta
        # Animation
        clean3D(ax,-5,5,-5,5,-8,2)
        print(f"rov.eta : {rov.eta}")
        draw_rov3D(ax,np.array([[x],[y],[z]]),eulermat(phi, theta, psi))
        #draw_cube3D(ax,np.array([[x],[y],[z]]),eulermat(phi, theta, psi),col='blue',size=5)
        #pause(dt)
    quit_pygame(running)
