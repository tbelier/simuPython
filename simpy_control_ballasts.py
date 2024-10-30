from sympy import sin, cos, Matrix, simplify,symbols,pi,hessian,lambdify,MatrixSymbol
from roblib import sawtooth

z,vz,theta,theta_dot,Vaeau,Vbeau,rho0,g,L,Jzz,Va0,Vb0,ma0,mb0=symbols("z vz theta theta_dot Vaeau Vbeau rho0 g L Jzz Va0 Vb0 ma0 mb0")
w=MatrixSymbol('w',2,1)
dw=MatrixSymbol('dw',2,1)
d2w=MatrixSymbol('d2w',2,1)
d3w=MatrixSymbol('d3w',2,1)

X=Matrix([z,vz,theta,theta_dot,Vaeau,Vbeau])

m0=ma0+mb0
dvz=g*(-1+(Va0+Vb0-(Vaeau+Vbeau))*rho0/m0)
dtheta_dot=g*(L/2)*cos(theta)/Jzz*(mb0-ma0+(Va0-Vb0-Vaeau+Vbeau)*rho0)

f=Matrix([vz, dvz, theta_dot, dtheta_dot, 0, 0])
g1=Matrix([0,0,0,0,1,0])
g2=Matrix([0,0,0,0,0,1])

h1=Matrix([z])
h2=Matrix([theta])

dh1=h1.jacobian(X)
Lfh1=dh1*f
d2h1=Lfh1.jacobian(X)
Lf2h1=d2h1*f
d3h1=Lf2h1.jacobian(X)
Lf3h1=d3h1*f
Lg1Lf2h1=d3h1*g1
Lg2Lf2h1=d3h1*g2

dh2=h2.jacobian(X)
Lfh2=dh2*f
d2h2=Lfh2.jacobian(X)
Lf2h2=d2h2*f
d3h2=Lf2h2.jacobian(X)
Lf3h2=d3h2*f
Lg1Lf2h2=d3h2*g1
Lg2Lf2h2=d3h2*g2

A=Matrix([[Lg1Lf2h1[0,0],Lg2Lf2h1[0,0]],[Lg1Lf2h2[0,0],Lg2Lf2h2[0,0]]])
fct_A_control=lambdify((z,vz,theta,theta_dot,Vaeau,Vbeau,rho0,g,L,Jzz,Va0,Vb0,ma0,mb0),A)

b=Matrix([Lf3h1,Lf3h2])
fct_b_control=lambdify((z,vz,theta,theta_dot,Vaeau,Vbeau,rho0,g,L,Jzz,Va0,Vb0,ma0,mb0),b)

y=Matrix([[z],[theta]])
dy=Matrix([[vz],[theta_dot]])
d2y=Matrix([[dvz],[dtheta_dot]])
v=(w-y)+3*(dw-dy)+3*(d2w-d2y)+d3w
#v=Matrix([v[0,0],sawtooth(v[1,0])])
fct_v_control=lambdify((z,vz,theta,theta_dot,Vaeau,Vbeau,rho0,g,L,Jzz,Va0,Vb0,ma0,mb0,w,dw,d2w,d3w),v)