syms phi l theta x_b z_b phi_dot l_dot theta_dot x_b_dot z_b_dot mb mw g Iw Rw real
%x=[phi,l,theta]
x_b=Rw*phi+l*sin(theta);

z_b=Rw+l*cos(theta);

x_b_dot=Rw*phi_dot+l_dot*sin(theta)+l*cos(theta)*theta_dot;

z_b_dot=l_dot*cos(theta)-l*sin(theta)*theta_dot;

T=1/2*(Iw*phi_dot^2+mw*Rw^2*phi_dot^2+mb*x_b_dot^2+mb*z_b_dot^2);
U=mw*g*Rw+mb*g*z_b;

L=T-U;
L=simplify(L);

T=collect(T,phi_dot^2);
T=collect(T,l_dot^2);
T=collect(T,theta_dot^2);

%inertia matrix 
M(1,1)=diff(T,phi_dot,2);
M(2,2)=diff(T,l_dot,2);
M(3,3)=diff(T,theta_dot,2);
Temp12=diff(T,phi_dot);
M(1,2)=diff(Temp12,l_dot);
M(2,1)=M(1,2);
Temp13=diff(T,phi_dot);
M(1,3)=diff(Temp13,theta_dot);
M(3,1)=M(1,3);
Temp23=diff(T,l_dot);
M(2,3)=diff(Temp23,theta_dot);
M(3,2)=M(2,3);
M=simplify(M);

%coriolis term
q=[phi;l;theta];
M1=M(:,1);
M2=M(:,2);
M3=M(:,3);
C1=(1/2)*(jacobian(M1,q)+jacobian(M1,q)'-diff(M,q(1)));
C2=(1/2)*(jacobian(M2,q)+jacobian(M2,q)'-diff(M,q(2)));
C3=(1/2)*(jacobian(M3,q)+jacobian(M3,q)'-diff(M,q(3)));
dq=[phi_dot;l_dot;theta_dot];
c1=dq'*C1*dq;
c2=dq'*C2*dq;
c3=dq'*C3*dq;
c=[c1;c2;c3];
c=simplify(c)


G=jacobian(U,q)'