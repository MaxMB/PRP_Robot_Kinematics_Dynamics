close, clear, clc;
%%%% Direct Kinematics %%%%
g = sym('g', 'real');     % Acceleration of gravity
h = sym('h', 'real');     % heigth
dh = sym('dh', 'real');   % d/dt (h)
ddh = sym('ddh', 'real'); % d^2/dt^2 (h)
t = sym('t', 'real');     % theta
dt = sym('dt', 'real');   % d/dt (theta)
ddt = sym('ddt', 'real'); % d^2/dt^2 (theta)
r = sym('r', 'real');     % radial distance
dr = sym('dr', 'real');   % d/dt (r)
ddr = sym('ddr', 'real'); % d^2/dt^2 (r)
DG1 = sym('DG1', 'real'); % Distance of the center of mass of LINK 1 to FRAME 1
DG3 = sym('DG3', 'real'); % Distance of the center of mass of LINK 3 to FRAME 3
DG4 = sym('DG4', 'real'); % Distance of the center of mass of LINK 4 to FRAME 4

% Angular velocity of FRAME i relative to FRAME i-1
w_r = @(R,w,dq) transpose(R) * w + [0;0;dq]; % Rotation
w_p = @(R,w) transpose(R) * w;               % Prismatic

% Angular acceleration of FRAME i relative to FRAME i-1
dw_r = @(R,dw,w,ddq,dq) transpose(R)*dw + [0;0;ddq] + cross(transpose(R)*w,[0;0;dq]);
dw_p = @(R,dw) transpose(R) * dw;

% Linear acceleration of FRAME i relative to FRAME i-1
ddp_r = @(R,dw,w,ddp,ph) transpose(R) * (ddp + cross(dw,ph) + cross(w,cross(w,ph)));
ddp_p = @(R,dw,w,ddp,ph,ddq,dq) transpose(R) * (ddp + cross(dw,ph) ...
    + cross(w,cross(w,ph))) + 2*cross(transpose(R)*w,[0;0;dq]) + [0;0;ddq];

% Acceleration of the center of mass of LINK i relative to FRAME i
dds = @(ddp,dw,w,sh) ddp + cross(dw,sh) + cross(w,cross(w,sh));

%%% FRAME 1 -> Base
w1_1 = [0;0;0];
dw1_1 = [0;0;0];
ddp1_1 = [0;0;g]; % gravity effect
sh1_1 = [0;0;DG1]; % Position of G1 relative to FRAME 1
dds1_1 = dds (ddp1_1, dw1_1, w1_1, sh1_1); % Acceleration of G1

%%% FRAME 2 -> Joint 12 is prismatic
R21 = eye(3); % Rotation matrix between FRAME i and FRAME i-1
ph21 = [0;0;h]; % Position of FRAME i relative to FRAME i-1
w2_2 = w_p (R21, w1_1);
dw2_2 = dw_p (R21, dw1_1);
ddp2_2 = ddp_p (R21, dw1_1, w1_1, ddp1_1, ph21, ddh, dh);
sh2_2 = [0;0;0];
dds2_2 = dds (ddp2_2, dw2_2, w2_2, sh2_2);

%%% FRAME 3 -> Joint 23 is rotational
R32 = [cos(t), -sin(t),  0;
       sin(t),  cos(t),  0;
            0,       0,  1];
ph32 = [0;0;0];
w3_3 = w_r (R32, w2_2, dt);
dw3_3 = dw_r (R32, dw2_2, w2_2, ddt, dt);
ddp3_3 = ddp_r (R32, dw2_2, w2_2, ddp2_2, ph32);
sh3_3 = [0;DG3;0];
dds3_3 = dds (ddp3_3, dw3_3, w3_3, sh3_3);

%%% FRAME 4 -> Joint 34 is prismatic
R43 = [1,  0, 0;
       0,  0, 1;
       0, -1, 0];
ph43 = [0;r;0];
w4_4 = w_p (R43, w3_3);
dw4_4 = dw_p (R43, dw3_3);
ddp4_4 = ddp_p (R43, dw3_3, w3_3, ddp3_3, ph43, ddr, dr);
sh4_4 = [0;0;-DG4];
dds4_4 = dds (ddp4_4, dw4_4, w4_4, sh4_4);

%%% FRAME 5 -> Object
R54 = eye(3);
ph54 = [0;0;0];
w5_5 = w_p (R54, w4_4);
dw5_5 = dw_p (R54, dw4_4);
ddp5_5 = ddp_r (R54, dw4_4, w4_4, ddp4_4, ph54);
sh5_5 = [0;0;0];
dds5_5 = dds (ddp5_5, dw5_5, w5_5, sh5_5);




%%%% Inverse Dynamics %%%%
m1 = sym('m1', 'real'); % Mass of LINK i
m2 = sym('m2', 'real');
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
Ixx1_1 = sym('Ixx1_1', 'real');
Iyy1_1 = sym('Iyy1_1', 'real');
Izz1_1 = sym('Izz1_1', 'real');
Ixy1_1 = sym('Ixy1_1', 'real');

% Force and Torque
f = @(R,fa,ddsi,m) R * fa + m * ddsi;
n = @(R,na,I,dw,w,m,sh,ddsi,ph,fa) R*na + I*dw + cross(w,I*w) ...
    + m*cross(sh,ddsi) + cross(ph,R*fa);

%%% FRAME 5 -> Object -> Material Point
I5_5 = zeros(3); % Inertia Matrix
f5_5 = f (eye(3), [0;0;0], dds5_5, m5);
n5_5 = n (eye(3), [0;0;0], I5_5, dw5_5, w5_5, m5, sh5_5, dds5_5, [0;0;0], [0;0;0]);

%%% FRAME 4
I4_4 = [Ixx4_4, Ixy4_4,      0;
        Ixy4_4, Iyy4_4,      0;
             0,      0, Izz4_4];
f4_4 = f (R54, f5_5, dds4_4, m4);
n4_4 = n (R54, n5_5, I4_4, dw4_4, w4_4, m4, sh4_4, dds4_4, ph54, f5_5);

%%% FRAME 3
I3_3 = [Ixx3_3,      0, Ixz3_3;
             0, Iyy3_3,      0;
        Ixz3_3,      0, Izz3_3];
f3_3 = f (R43, f4_4, dds3_3, m3);
n3_3 = n (R43, n4_4, I3_3, dw3_3, w3_3, m3, sh3_3, dds3_3, ph43, f4_4);

%%% FRAME 2
I2_2 = [Ixx2_2, Ixy2_2,      0;
        Ixy2_2, Iyy2_2,      0;
             0,      0, Izz2_2];
f2_2 = f (R32, f3_3, dds2_2, m2);
n2_2 = n (R32, n3_3, I2_2, dw2_2, w2_2, m2, sh2_2, dds2_2, ph32, f3_3);

%%% FRAME 1 -> Base
I1_1 = [Ixx1_1, Ixy1_1,      0;
        Ixy1_1, Iyy1_1,      0;
             0,      0, Izz1_1];
f1_1 = f (R21, f2_2, dds1_1, m1);
n1_1 = n (R21, n2_2, I1_1, dw1_1, w1_1, m1, sh1_1, dds1_1, ph21, f2_2);

% Organizing
co =  [ddh,dh,ddt,dt,ddr,dr];
f1_1 = collect(f1_1, co);   n1_1 = collect(n1_1, co);
f2_2 = collect(f2_2, co);   n2_2 = collect(n2_2, co);
f3_3 = collect(f3_3, co);   n3_3 = collect(n3_3, co);
f4_4 = collect(f4_4, co);   n4_4 = collect(n4_4, co);
f5_5 = collect(f5_5, co);   n5_5 = collect(n5_5, co);