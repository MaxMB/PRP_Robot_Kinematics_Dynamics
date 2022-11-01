close, clear, clc;
%%%% Direct Kinematics %%%%
g = sym('g', 'real');       % Acceleration of gravity
h = sym('h', 'real');       % heigth
dh = sym('dh', 'real');     % d/dt (h)
ddh = sym('ddh', 'real');   % d^2/dt^2 (h)
th = sym('th', 'real');     % theta
dth = sym('dth', 'real');   % d/dt (theta)
ddth = sym('ddth', 'real'); % d^2/dt^2 (theta)
r = sym('r', 'real');       % radial distance
dr = sym('dr', 'real');     % d/dt (r)
ddr = sym('ddr', 'real');   % d^2/dt^2 (r)
DG3 = sym('DG3', 'real');   % Distance of the center of mass of LINK 3 to FRAME 2
DG4 = sym('DG4', 'real');   % Distance of the center of mass of LINK 4 to FRAME 5
q = [h; th; r];             % Generalized coordinates
dq = [dh; dth; dr];         % Generalized velocities
ddq = [ddh; ddth; ddr];     % Generalized acceleration

%%% Position vector of the centers of mass relative to FRAME 1
rG2_1 = [0; 0; h];
rG3_1 = [-DG3*sin(th); DG3*cos(th); h];
rG4_1 = [-(r-DG4)*sin(th); (r-DG4)*cos(th); h];
rG5_1 = [-r*sin(th); r*cos(th); h];
rG_1 = [rG2_1; rG3_1; rG4_1; rG5_1];

%%% Linear velocity vector of the centers of mass relative to FRAME  1
Jv = jacobian(rG_1, q);
vG_1 = Jv * dq;
vG2_1 = vG_1(1:3,1);
vG3_1 = vG_1(4:6,1);
vG4_1 = vG_1(7:9,1);
vG5_1 = vG_1(10:12,1);

%%% Linear acceleration vector of the centers of mass relative to FRAME  1
a = sym('h(t)', 'real');
b = sym('th(t)', 'real');
c = sym('r(t)', 'real');
Ja = diff(subs(Jv, [h,th,r], [a,b,c]));
Ja = subs(Ja, [a,b,c,diff(a),diff(b),diff(c)], [h,th,r,dh,dth,dr]);
aG_1 = Ja * dq + Jv * ddq;
aG2_1 = aG_1(1:3,1);
aG3_1 = aG_1(4:6,1);
aG4_1 = aG_1(7:9,1);
aG5_1 = aG_1(10:12,1);

%%% Angular velocity vector of the centers of mass relative to FRAME  1
w1_1 = [0;0;0];
w2_2 = [0;0;0];
w3_3 = [0;0;dth];
w4_4 = [0;0;0];
R32 = [cos(th),-sin(th),0; sin(th),cos(th),0; 0,0,1];
R43 = [1,0,0; 0,0,1; 0,-1,0];
R21 = eye(3);
R31 = R21 * R32;
R41 = R31 * R43;
w2_1 = w1_1 + R21 * w2_2;
w3_1 = w2_1 + R31 * w3_3;
w4_1 = w3_1 + R41 * w4_4;

%%% Angular acceleration vector of the centers of mass relative to FRAME  1
dw1_1 = [0;0;0];
dw2_2 = [0;0;0];
dw3_3 = [0;0;ddth];
dw4_4 = [0;0;0];
dw2_1 = dw1_1 + R21 * dw2_2 + cross(w1_1, R21 * w2_2);
dw3_1 = dw2_2 + R31 * dw3_3 + cross(w2_2, R31 * w3_3);
dw4_1 = dw3_3 + R41 * dw4_4 + cross(w3_3, R41 * w4_4);



%%%% Inverse Dynamics %%%%
m2 = sym('m2', 'real'); % Mass of LINK i
m3 = sym('m3', 'real');
m4 = sym('m4', 'real');
m5 = sym('m5', 'real');
Ixx4_4 = sym('Ixx4_4', 'real'); % Inertia matrix parameters
Iyy4_4 = sym('Iyy4_4', 'real');
Izz4_4 = sym('Izz4_4', 'real');
Ixy4_4 = sym('Ixy4_4', 'real');
Ixx3_3 = sym('Ixx3_3', 'real');
Iyy3_3 = sym('Iyy3_3', 'real');
Izz3_3 = sym('Izz3_3', 'real');
Ixz3_3 = sym('Ixz3_3', 'real');
Ixx2_2 = sym('Ixx2_2', 'real');
Iyy2_2 = sym('Iyy2_2', 'real');
Izz2_2 = sym('Izz2_2', 'real');
Ixy2_2 = sym('Ixy2_2', 'real');
Fm1 = sym('Fm1', 'real'); % Motors forces and torques
Tm2 = sym('Tm2', 'real');
Fm3 = sym('Fm3', 'real');

%%% Matrix C
C = eye(3); % n = m = 3

%%% Matrix D
JV2_1 = jacobian([vG2_1',w2_1'], dq);
JV3_1 = jacobian([vG3_1',w3_1'], dq);
JV4_1 = jacobian([vG4_1',w4_1'], dq);
D = [JV2_1', JV3_1', JV4_1'];

% Inertia Matrices
I2_2 = [Ixx2_2, Ixy2_2,      0;
        Ixy2_2, Iyy2_2,      0;
             0,      0, Izz2_2];
I3_3 = [Ixx3_3,      0, Ixz3_3;
             0, Iyy3_3,      0;
        Ixz3_3,      0, Izz3_3];
I4_4 = [Ixx4_4, Ixy4_4,      0;
        Ixy4_4, Iyy4_4,      0;
             0,      0, Izz4_4];

%%% Vector F
F5 = m5 * aG5_1;                 % Object reaction force
M5 = cross(R41 * [0;0;DG4], F5); % Object reaction torque 
F2 = [0;0;Fm1-m2*g] - m2*aG2_1;
M2 = [0;0;0] - I2_2*dw2_1 - cross(w2_1,I2_2*w2_1);
F3 = [0;0;-m3*g] - m3*aG3_1;
M3 = [0;0;Tm2] - I3_3*dw3_1 - cross(w3_1,I3_3*w3_1);
F4 = R41*[0;(m4+m5)*g;Fm3] - F5 - m4*aG4_1;
M4 = - M5 - I4_4*dw4_1 - cross(w4_1,I4_4*w4_1);
F = [F2; M2; F3; M3; F4; M4];

%%% Gibbs-Appel Equations
GAE = C' * D * F == zeros(3,1);
GAE = collect(GAE, [ddh,dh,ddth,dth,ddr,dr]); % Organize
GAE = simplify(GAE); % Cancel terms
GAE = collect(GAE, [ddh,dh,ddth,dth,ddr,dr]);