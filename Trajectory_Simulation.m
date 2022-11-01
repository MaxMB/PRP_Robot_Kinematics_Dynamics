clear, close, clc;
%Definicao dos Parametros
T = 10;    %tempo final [s]
x1 = 200;  %[mm]
y1 = -100; %[mm]
z1 = 0;    %[mm]
x2 = 200;  %[mm]
y2 = 100;  %[mm]
z2 = 50;   %[mm]
L = sqrt(((x2-x1)^2)+((y2-y1)^2)+((z2-z1)^2));

%Equacoes de movimento
t = 0:0.1:T;
y = L*(6*((t/T).^5)-15*((t/T).^4)+10*((t/T).^3));
dy = (L/T)*(30*((t/T).^4)-60*((t/T).^3)+30*((t/T).^2));
ddy = (L/T.^2)*(120*((t/T).^3)-180*((t/T).^2)+60*((t/T)));

%Plotando Grafico
plot(t,y,'b',t,dy,'r',t,ddy,'k');
title('Trajetória do objeto carregado');
xlabel('Tempo [s]');
ylabel('Intensidade das grandezas');
legend('Posição [mm]','Velocidade [mm/s]','Aceleração [mm/s^2]','Location','northwest');
grid minor;