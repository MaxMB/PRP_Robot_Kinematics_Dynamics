close, clear, clc;
%%%% Dynamics %%%%
m2 = 2.78289 + 0.14835; % Atuador_P1_Base_R + Step_Motor_Medium [kg]
m3 = 0.62267 + 0.06642; % Atuador_R_Base_P2 + Step_Motor_Small [kg]
m4 = 0.21425; % Atuador_P2 [kg]
m_obj = 1; % Objeto carregado [kg]
Izz3 = 3972.41766 + 402.323635; % Atuador_R_Base_P2 + Step_Motor_Small [kg*mm2]
Lzz4 = 369.70861; % Atuador_P2 [kg*mm2]
g = 9806.65; % aceleração da gravidade [mm/s2]

D_G4_obj = 120 - 70.23; % Dist(centro de massa garra, objeto) [mm]
D_G4 = r - D_G4_obj; % Dist(centro de massa garra, eixo de rotação) [mm]
D_G4_2 = D_G4 .^ 2;
D_obj = r; % Distância do objeto em relação ao eixo de rotação [mm]
D_obj_2 = r .^ 2;

Fm1 = (m2 + m3 + m4 + m_obj) * (ddh + g) * 1e-3;
Tm2 = ((Izz3 + Lzz4 + m4 * D_G4_2 + m_obj * D_obj_2) .* ddtheta ...
    + 2 * (m4 * D_G4 + m_obj * D_obj) .* dtheta .* dr) * 1e-6;
Fm3 = ((m4 + m_obj) * ddr - (m4 * D_G4 + m_obj * D_obj) .* dtheta.^2) * 1e-3;

figure(1);
subplot(311), plot(t, Fm1), grid minor, title('Força do Motor 1');
xlabel('Tempo [s]'), ylabel('Força [N]');
subplot(312), plot(t, Tm2), grid minor, title('Torque do Motor 2');
xlabel('Tempo [s]'), ylabel('Torque [N*m]');
subplot(313), plot(t, Fm3), grid minor, title('Força do Motor 3');
xlabel('Tempo [s]'), ylabel('Força [N]');