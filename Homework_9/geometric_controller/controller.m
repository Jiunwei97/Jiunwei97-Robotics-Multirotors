function out = controller(u,P)

% input(25*1):desired trajectory and full state feedback, x v R Omega time
% output(4*1): force and moment control input

% process inputs
xd    = u(1:3);
b1d   = u(4:6);
%if wants to use b1d ->>position controller?

% current state
x     = u(7:9);
v     = u(10:12);
R     = reshape(u(13:21),3,3);
Omega = u(22:24);
t     = u(end);

%%initial:
Rd=eye(3);
%xd = [0.0 0.0 2.0];
Omega_d=[0;0.1;1];
%Rd*xd == [0 0 2];

%---------attitude controller formula---------
Rd_dot=Rd*hat(Omega_d);
%the error of attitude formula:
ex=0.5*vee(transpose(Rd)*R-transpose(R)*Rd);
%the error of angular velocity formula:
eo=Omega-transpose(R)*Rd*Omega_d;
e3 = [0;0;1];
%eR=Rd-R;

ep=xd-x;
%ev=xddot-xdot;

J = diag([P.Jxx P.Jyy P.Jzz]);
M = -P.kR * ex- P.kOmega* eo + cross(Omega, J*Omega);
%f = (P.kx*(x(3)-xd(3))+P.kv*(xdot(3)-xddot(3))+P.mass*P.gravity)/dot(e3,R*e3);
f = (P.kx*(x(3)-xd(3))+P.mass*P.gravity)/dot(e3,R*e3);
%f=(P.kx*ep+P.kv*ev+P.mass*P.gravity*zw-P.mass*xdotdot)*R*e3;
out = [f;M];
end