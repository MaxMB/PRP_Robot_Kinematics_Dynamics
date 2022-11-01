close, clc;
%%%% Inverse Dynamics %%%%
m2 = 2.78289 + 0.14835; % Atuador_P1_Base_R + Step_Motor_Medium [kg]
m3 = 0.62267 + 0.06642; % Atuador_R_Base_P2 + Step_Motor_Small [kg]
m4 = 0.2142522; % Atuador_P2 [kg]
m5 = 1; % Objeto carregado [kg]
Lzz3_3 = 3972.41766 + 402.323635; % Atuador_R_Base_P2 + Step_Motor_Small [kg*mm2]
Iyy4_4 = 369.7086102; % Atuador_P2 [kg*mm2]
g = 9806.65; % acelera��o da gravidade [mm/s2]
DG4 = 120 - 70.2348; % Dist(G4, objeto) [mm]

Fm1 = (m2 + m3 + m4 + m5) * (ddh + g) * 1e-3;
Tm2 = ((Lzz3_3 + Iyy4_4 + m4*(r-DG4).^2 + m5*r.^2) .* ddtheta ...
    + 2 * (m4*(r-DG4) + m5*r) .* dtheta .* dr) * 1e-6;
Fm3 = ((m4 + m5) * ddr - (m4*(r-DG4) + m5*r) .* dtheta.^2) * 1e-3;

figure(1);
subplot(311), plot(t, Fm1), grid minor, title('For�a do Motor 1');
xlabel('Tempo [s]'), ylabel('For�a [N]');
subplot(312), plot(t, Tm2), grid minor, title('Torque do Motor 2');
xlabel('Tempo [s]'), ylabel('Torque [N\cdotm]');
subplot(313), plot(t, Fm3), grid minor, title('For�a do Motor 3');
xlabel('Tempo [s]'), ylabel('For�a [N]');