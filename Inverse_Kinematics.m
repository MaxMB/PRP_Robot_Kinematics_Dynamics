close, clear, clc;
Pi = [200; -100; 0]; % Posição inicial da trajetória [mm]
Pf = [200; 100; 50]; % Posição final da trajetória [mm]
L = norm(Pf - Pi); % Comprimento da trajetória [mm]
v = (Pf - Pi) / L; % Vetor unitário da mesma orientação e sentido da trajetória
T = 10; % Tempo total [s]
n = 100; % Número de passos
t = linspace(0, T, n); % Vetor do tempo
x0 = [224; -0.5; 0]; % Vetor da posição inicial

% Polinômio interpolador de 5º grau e suas derivadas
y = L * (6*(t/T).^5 - 15*(t/T).^4 + 10*(t/T).^3);
dy = L / T * (30*(t/T).^4 - 60*(t/T).^3 + 30*(t/T).^2);
ddy = L / T^2 * (120*(t/T).^3 - 180*(t/T).^2 + 60*t/T);

r_P = zeros(3,n);
Mat_r = zeros(3,n);
Mat_v = zeros(3,n);
Mat_a = zeros(3,n);

for i = 1:n
    % Posições
    r_P(:,i) = Pi + y(i) * v;
    P = @(x) [x(1) * cos(x(2)) - r_P(1,i);
              x(1) * sin(x(2)) - r_P(2,i);
                          x(3) - r_P(3,i)];
    Mat_r(:,i) = fsolve(P, x0); clc;
    r = Mat_r(1,:);
    theta = Mat_r(2,:);
    h = Mat_r(3,:);
    x0 = [r(i); theta(i); h(i)]; 
    
    % Velocidades
    v_P = dy(i) * v;
    Mat_A = [cos(theta(i)), -r(i)*sin(theta(i)),    0;
             sin(theta(i)),  r(i)*cos(theta(i)),    0;
                         0,                   0,    1];
    Mat_v(:,i) = Mat_A \ v_P;
    dr = Mat_v(1,:);
    dtheta = Mat_v(2,:);
    dh = Mat_v(3,:);
    
    % Acelerações
    a_P = ddy(i) * v;
    Mat_B = [2*dr(i)*sin(theta(i))*dtheta(i) + r(i)*cos(theta(i))*dtheta(i)^2 + a_P(1);
            -2*dr(i)*cos(theta(i))*dtheta(i) + r(i)*sin(theta(i))*dtheta(i)^2 + a_P(2);
                                                                                a_P(3)];
    Mat_a(:,i) = Mat_A \ Mat_B;
    ddr = Mat_a(1,:);
    ddtheta = Mat_a(2,:);
    ddh = Mat_a(3,:);
end