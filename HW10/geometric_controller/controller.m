function out = controller(u,P)

% input(25*1):desired trajectory and full state feedback, x v R Omega time
% output(4*1): force and moment control input

% process inputs
xd    = u(1:3);
b1d   = u(4:6);

% current state
x     = u(7:9);
v     = u(10:12);
R     = reshape(u(13:21),3,3);
Omega = u(22:24);
t     = u(end);

%============initialize=================%
xd = [sin(t); cos(t); 1];
xddot=[cos(t);-sin(t);0];
xddoubledot=[-sin(t);-cos(t);0];
J = diag([P.Jxx P.Jyy P.Jzz]);
e3 = [0;0;1];
ex=xd-x;
ev=xddot-v;
Fdes = (-P.kx*ex-P.kv*ev-P.mass*P.gravity*e3+P.mass*xddoubledot);
%%inertia frame%%
b3c=-Fdes/normalize(Fdes);
b2c=cross(b3c,b1d)/normalize(cross(b3c,b1d));
b1c=cross(b2c,b3c);
Rc=[b1c;cross(b3c,b1c);b3c];
eR=0.5*vee(transpose(Rc)*R-transpose(R)*Rc);
Omega_c=vee(transpose(Rc)*Rc_dot);
eo=Omega-transpose(R)*Rc*Omega_c;
hat(Omega_c)=transpose(Rc)*Rc_dot;

f = dot((P.kx*ex+P.kv*ev+P.mass*P.gravity*e3-P.mass*xddoubledot),R*e3);
M = -P.kR * eR- P.kOmega* eo + cross(Omega, J*Omega);
out = [f;M];
end