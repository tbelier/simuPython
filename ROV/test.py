import sys
import os

parent_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
sys.path.append(parent_dir)

from roblib import *
from manette import *
from T200 import T200

class ROV:

    def __init__(self, eta0, nu0, nup0):
        #Initialisation des paramètres :
        self.a=5

        #Rigid body dynamics parameter
        self.m = 11.5 #(kg)
        self.W = 112.8 #(N)
        self.B = 114.8 #(N)
        self.rb = np.array([[0], [0], [0]]) #(m)
        self.rg = np.array([[0], [0], [0.02]]) #(m)
        self.Ix = 0.16 #(kg m2)
        self.Iy = 0.16 #(kg m2)
        self.Iz = 0.16 #(kg m2)

        #Mass parameters
        self.Xup = -5.5     #Surge (kg)
        self.Yvp = -12.7    #Sway (kg)
        self.Zwp = -14.57   #Heave (kg)
        self.Kpp = -0.12    #Roll (kg m2/rad)
        self.Mqp = -0.12    #Pitch (kg m2/rad)
        self.Nrp = -0.12    #Yaw (kg m2/rad

        #Linear and quadratic damping parameters
        self.Xu,self.Xuu = -4.03,-18.18  # Surge (Ns/m) (Ns2/m2)
        self.Yv,self.Yvv = -6.22,-21.66  # Sway (Ns/m) (Ns2/m2)
        self.Zw,self.Zww = -5.18,-36.99  # Heave (Ns/m) (Ns2/m2)
        self.Kp,self.Kpp = -0.07,-1.55   # Roll (Ns/rad) (Ns2/rad2)
        self.Mq,self.Mqq = -0.07,-1.55   # Pitch (Ns/rad) (Ns2/rad2)
        self.Nr,self.Nrr = -0.07,-1.55   # Yaw (Ns/rad) (Ns2/rad2)

        #configuration matrix T
        
        self.T= np.array([[0.707, 0.707, -0.707, -0.707,0,0,0,0],
                          [-0.707,0.707,-0.707,0.707,0,0,0,0],
                          [0,0,0,0,-1,1,1,-1],
                          [0.06,-0.06,0.06,-0.06,-0.218,-0.218,0.218,0.218],
                          [0.06,0.06,-0.06,-0.06,0.120,-0.120,0.120,-0.120],
                          [-0.1888,0.1888,0.1888,-0.1888,0,0,0,0]])
        
        self.K=np.diag((1,1,1,1,1,1,1,1))

        #Fossen's rigid body vectors
        self.eta = eta0 # N,E,D,eta,eps1,eps2,eps3
        self.etap = 0
        self.nu  = nu0 # u,v,w,p,q,r
        self.nup  = nup0 # derivative of : u,v,w,p,q,r

        #Récupération de la Look-up Table pour passer des PWM à la commande en force
        self.T200 = T200("/home/tbelier/Documents/GIT/python-simu/ROV/thruster.csv")


    


    def get_g(self):
        x,y,z,phi,theta,psi = self.eta.flatten()
        zg = self.rg[2,0]
        """
        N,E,D,n,eps1,eps2,eps3 = self.eta.flatten()
        g = np.array([[(self.B-self.W)*(2*eps1*eps3-2*eps2*n)],
                      [(self.B-self.W)*(2*eps2*eps3-2*eps1*n)],
                      [(self.W-self.B)*(2*eps1**2+2*eps2**2-1)],
                      [zg*self.W*(2*eps2*eps3+2*eps1*n)],
                      [zg*self.W*(2*eps1*eps3-2*eps2*n)]])
        """
        g = np.array([[(self.B-self.W)*sin(theta)],
                [(self.B-self.W)*cos(theta)*sin(phi)],
                [(self.W-self.B)*cos(theta)*cos(phi)],
                [zg*self.W*cos(theta)*sin(phi)],
                [zg*self.W*sin(theta)],
                [0]])
        return g

    def get_D(self):
        u,v,w,p,q,r = self.nu.flatten()

        D_L  = -np.diag([self.Xu, self.Yv, self.Zw, self.Kp, self.Mq, self.Nr])
        D_NL = -np.diag([self.Xuu*abs(u), self.Yvv*abs(v), self.Zww*abs(w), self.Kpp*abs(p), self.Mqq*abs(q), self.Nrr*abs(r)])

        return D_L+D_NL
    
    def get_M(self):
        zg = self.rg[2,0]
        M_RB=np.array([[self.m,0,0,0,self.m*zg,0],
                       [0,self.m,0,-self.m*zg,0,0],
                       [0,0,self.m,0,0,0],
                       [0,-self.m*zg,0,self.Ix,0,0],
                       [self.m*zg,0,0,0,self.Iy,0],
                       [0,0,0,0,0,self.Iz]])
        
        M_A=-np.diag([self.Xup,self.Yvp,self.Zwp,self.Kpp,self.Mqp,self.Nrp])
        
        return M_RB+M_A
    
    def get_C(self):
        u,v,w,p,q,r = self.nu.flatten()

        C_RB=np.array([[0,0,0,0,self.m*w,0],
                       [0,0,0,-self.m*w,0,0],
                       [0,0,0,self.m*v,-self.m*u,0],
                       [0,self.m*w,-self.m*v,0,self.Iz*r,-self.Iy*q],
                       [-self.m*w,0,-self.m*u,-self.Iz*r,0,self.Ix*p],
                       [self.m*v,-self.m*u,0,self.Iy*q,-self.Ix*p,0]])
        
        #TODO:vérifier qu'il n'y a pas de problème venant du fait que c'est self.Zwp alors que dans la thèse c'est écrit zwp (minuscule)
        C_A=np.array([[0,0,0,0,self.Zwp*w,0],
                      [0,0,0,-self.Zwp*w,0,-self.Xup*u],
                      [0,0,0,-self.Yvp*v,self.Xup*u,0],
                      [0,-self.Zwp*w,self.Yvp*v,0,-self.Nrp*r,self.Mqp*q],
                      [self.Zwp*w,0,-self.Xup*u,self.Nrp*r,0,-self.Kpp*p],
                      [-self.Yvp*v,self.Xup*u,0,-self.Mqp*q,self.Kpp*p,0]])


        if C_RB.shape == C_A.shape:
            return C_RB + C_A
        else:
            raise ValueError("Les matrices C_RB et C_A doivent avoir la même forme.")

        #return C_RB+C_A
  
    def get_motorForces(self,u):
        """
        prend en entrée u = [[u0], et renvoie en sortie tau = [[Fx],
                             [u1],                             [Fy],
                             [u2],                             [Fz],
                             [u3],                             [Fzly-Fylz],
                             [u4],                             [Fzly-Fylz],
                             [u5],                             [Fylx-Fxly]]
                             [u6],                             
                             [u7]]                             

        """
        #T=self.T
        #Tplus = T.T@inv(T@T.T)
        #u = Tplus@np.array([[0],[0],[0],[-1.08],[0],[0]])
        ##print(f"u : {u}")
        L_F = [[self.T200.get_force(u[k,0])] for k in range(0,len(u))]
        #L_tau = [[self.T200.get_torque(u[k,0])] for k in range(0,len(u))]
        #print(f"L_F : {L_F}")
        tau = self.T@L_F
        #print(f"tau : {tau}")
        return np.array([[0],[0],[100],[0],[0],[0]])
        #return tau
    
    def get_J(self): #depends on eta
        x,y,z,phi,theta,psi = self.eta.flatten()
        """
        Rphi = np.array([[1,0,0],
                         [0,cos(phi),-sin(phi)],
                         [0,sin(phi),cos(phi)]])
        
        Rtheta = np.array([[cos(theta),0,sin(theta)],
                         [0,1,0],
                         [-sin(theta),0,cos(theta)]])
        
        Rpsi = np.array([[cos(psi),-sin(psi),0],
                         [sin(psi),cos(psi),0],
                         [0,0,1]])
        
        R = Rpsi@Rtheta@Rphi
        """
        R=eulermat(phi,theta,psi)
        T = np.array([[1, sin(phi)*tan(theta), cos(phi)*tan(theta)],
                        [0, cos(phi), -sin(phi)],
                        [0, sin(phi) / cos(theta), cos(phi)/cos(theta)]])
        
        return np.vstack((np.hstack((R,            np.zeros((3,3)))),
                          np.hstack((np.zeros((3,3)),T))))


        """
        eps1,eps2,eps3,n = eta.flatten()
        R_q = np.array([[1-2*(eps2**2+eps3**2), 2*(eps1*eps2-eps3*n),2*(eps1*eps3-eps2*n)],
                        [[2*(eps1*eps2-eps3*n),1-2*(eps1**2+eps3**2), 2*(eps2*eps3-eps1*n)]],
                        [[2*(eps1*eps3-eps2*n), 2*(eps2*eps3-eps1*n), 1-2*(eps1**2+eps2**2)]]])
        
        T_q = np.array([[-eps1,-eps2,-eps3],
                        [n, -eps3, eps2],
                        [eps2,n,-eps1],
                        [-eps2,eps1,n]])/2
        
        return np.vstack((np.hstack(R_q,            np.zeros((3,3))),
                          np.hstack(np.zeros((4,3)),T_q)))
        """

    def get_J_1_p(self):
        x,y,z,phi,theta,psi = self.eta.flatten()
        etap = self.get_J()@self.nu
        xp,yp,zp,phip,thetap,psip = etap.flatten()
        #R=Rpsi@Rtheta@Rphi so inv(R)=R.T=Rphi.T@Rtheta.T@R.psi.T
        Rphi,Rtheta,Rpsi = expw([phi,0,0]), expw([0,theta,0]), expw([0,0,psi])
        """
        Rphi = np.array([[1,0,0],
                    [0,cos(phi),-sin(phi)],
                    [0,sin(phi),cos(phi)]])
        
        Rtheta = np.array([[cos(theta),0,sin(theta)],
                         [0,1,0],
                         [-sin(theta),0,cos(theta)]])
        
        Rpsi = np.array([[cos(psi),-sin(psi),0],
                         [sin(psi),cos(psi),0],
                         [0,0,1]])
        """
        R_1_p = Rphi.T@Rtheta.T@Rpsi.T
        
        T_1_p = np.array([[0,0,-thetap*cos(theta)],
                          [0,-thetap*sin(theta),-thetap*sin(theta)*sin(phi)+phip*cos(theta)*cos(phi)],
                          [0,-thetap*cos(phi),-thetap*sin(theta)*cos(phi)-phip*cos(theta)*sin(phi)]]) 
        Jp = np.vstack((np.hstack((R_1_p,            np.zeros((3,3)))),
                    np.hstack((np.zeros((3,3)),T_1_p))))
        #print(Jp)
        return Jp


    def f(self,u): #fonction qui permet de faire évoluer le système !
        """
        permet de calculer la dérivée seconde eta, elle prend en entrée les consignes ainsi que le vecteur d'état X=(eta,nu,nup)

        """
        J = self.get_J()
        J_1_p = self.get_J_1_p()
        
        C = self.get_C()
        D = self.get_D()
        g = self.get_g()
        M = self.get_M()
        tau = self.get_motorForces(u)
        
        nu = self.nu
        etap = J@self.nu #ATTENTION ICI JE N'AI PAS ETAP ?
        etapp = J@(inv(M)@(tau-C@nu-D@nu-g)-J_1_p@etap)
        
        return etapp
    
    #============================ TODO :
    def control(x,ax_bar, wr_bar):
        v=x[6]
        u0=np.sqrt((ax_bar+k2*v**2)/k1)
        u123=inv(B)@wr_bar/v
        u=vstack((u0,u123))
        return u

    def integrate(self,etapp,dt):
        """
        permet d'intégrer etapp pour actualiser nos variables d'état
        """
        
        #etap = ODE(etapp)
        self.etap, self.eta = self.etap + dt*etapp, self.eta + dt*self.etap
        #print(self.eta)
        #eta = ODE(etap)
        self.nu = self.get_J()@self.etap

        return self.eta,self.nu,self.etap

