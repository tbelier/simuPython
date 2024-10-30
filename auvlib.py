from roblib import * 

def draw_rov2D(x,colbuoy="blue",colcenter='black',r=1,w=2):
    mx,my,θ=tolist(x)[0:3]
    delta_y = 2.5
    M = r*array([[0,0,-1,-1,-2,-2,-1,-1,1,1,2,2,1,1,1,-1], [0+delta_y,-1+delta_y,-1+delta_y,0+delta_y,-1+delta_y,-4+delta_y,-5+delta_y,-1+delta_y,-1+delta_y,0+delta_y,-1+delta_y,-4+delta_y,-5+delta_y,-1+delta_y,-4+delta_y,-4+delta_y]])
    Mleft = r*array([[-1,-2, -2, -1,-1], [0+delta_y,-1+delta_y,-4+delta_y,-5+delta_y,0+delta_y]])
    Mright = r*array([[1,2, 2, 1,1], [0+delta_y,-1+delta_y,-4+delta_y,-5+delta_y,0+delta_y]])
    M=add1(M)
    Mleft = add1(Mleft)
    Mright = add1(Mright)
    plot2D(tran2H(mx,my)@rot2H(θ)@M,colcenter,w)
    plot2D(tran2H(mx,my)@rot2H(θ)@Mleft,colbuoy,w)
    plot2D(tran2H(mx,my)@rot2H(θ)@Mright,colbuoy,w)

def draw_circus(x,L,l,col='gray',r=1,w=3):
    mx,my,θ=tolist(x)[0:3]
    delta_y = 2.5
    Mtop = r*array([[-L/2,L/2], [l/2,l/2]])
    McircleR = r*array([[L/2+l/2*np.cos(theta) for theta in np.linspace(np.pi/2, -np.pi/2, num=100)],[l/2*np.sin(theta) for theta in np.linspace(-np.pi/2, np.pi/2, num=100)]])
    Mbot = r*array([[-L/2,L/2], [-l/2,-l/2]])
    McircleL = r*array([[-L/2+l/2*np.cos(theta) for theta in np.linspace(np.pi/2,np.pi+np.pi/2, num=100)],[l/2*np.sin(theta) for theta in np.linspace(np.pi/2,np.pi+np.pi/2, num=100)]])
    Mtop=add1(Mtop)
    McircleR=add1(McircleR)
    Mbot=add1(Mbot)
    McircleL=add1(McircleL)
    plot2D(tran2H(mx,my)@rot2H(θ)@Mtop,col,w)
    plot2D(tran2H(mx,my)@rot2H(θ)@McircleR,col,w)
    plot2D(tran2H(mx,my)@rot2H(θ)@Mbot,col,w)
    plot2D(tran2H(mx,my)@rot2H(θ)@McircleL,col,w)

def draw_cylinder(x,r,L,col="gray",scale=1,width=2):
    mx,my,θ=tolist(x)[0:3]
    delta_y = 2.5
    M = scale*array([[mx + r*np.cos(dtheta) for dtheta in np.linspace(0,2*np.pi,num=10)],[my + r*np.sin(dtheta) for dtheta in np.linspace(0,2*np.pi,num=10)]])
    M=add1(M)

    plot2D(tran2H(mx,my)@rot2H(θ)@M,col,width)

def draw_robot3D(ax,p,R,col='blue',size=1):
    M=tran3H(*p[0:3,0])@diag([size,size,size,1])@ ToH(R) @ auv3H()
    draw3H(ax, M, col, True, 1)
    pause(0.001)

def auv_animation(w,h):    
    ax=init_figure(-w,w,-h,h)
    for t in arange(0,5,0.1) :
        clear(ax)
        draw_cylinder(array([[1],[0],[0],[0],[0]]),1,1,'blue')
    show()


def circus_animation(w,h):    
    ax=init_figure(-w,w,-h,h)
    for t in arange(0,5,0.1) :
        clear(ax)
        draw_rov2D(array([[t],[0],[0],[0],[0]]),'blue')
        draw_circus(array([[0],[0],[0],[0],[0]]), 100,50)
    show()




if __name__ == "__main__":
    auv_animation(150,150)
    #circus_animation(150,150)