close, clc;
figure('Name','Coordenada generalizada r','NumberTitle','off')
subplot(3,1,1), plot(t,r,'linewidth',1.5), grid minor
title('\textbf{Coordenada generalizada $r$ e suas derivadas}','Interpreter', 'latex')
xlabel('$Tempo [s]$','Interpreter','latex')
ylabel('$r [mm]$','Interpreter','latex')
set(gca,'Fontsize',12,'FontName','Times New Roman')
subplot(3,1,2), plot(t,dr,'linewidth',1.5), grid minor
xlabel('$Tempo [s]$','Interpreter','latex')
ylabel('$\dot{r} [mm/s]$','Interpreter','latex')
set(gca,'Fontsize',12,'FontName','Times New Roman')
subplot(3,1,3), plot(t,ddr,'linewidth',1.5), grid minor
xlabel('$Tempo [s]$','Interpreter', 'latex')
ylabel('$\ddot{r} [mm/s^2]$','Interpreter', 'latex')
set(gca,'Fontsize',12,'FontName', 'Times New Roman')

figure('Name','Coordenada generalizada theta','NumberTitle','off')
subplot(3,1,1), plot(t,theta*180/pi,'linewidth',1.5), grid minor
title('\textbf{Coordenada generalizada $\theta$ e suas derivadas}','Interpreter', 'latex')
xlabel('$Tempo [s]$','Interpreter','latex')
ylabel('$\theta [deg]$','Interpreter','latex')
set(gca,'Fontsize',12,'FontName','Times New Roman')
subplot(3,1,2), plot(t,dtheta*180/pi,'linewidth',1.5), grid minor
xlabel('$Tempo [s]$','Interpreter','latex')
ylabel('$\dot{\theta} [deg/s]$','Interpreter','latex')
set(gca,'Fontsize',12,'FontName','Times New Roman')
subplot(3,1,3), plot(t,ddtheta*180/pi,'linewidth',1.5), grid minor
xlabel('$Tempo [s]$','Interpreter','latex')
ylabel('$\ddot{\theta} [deg/s^2]$','Interpreter','latex')
set(gca,'Fontsize',12,'FontName','Times New Roman')

figure('Name','Coordenada generalizada h','NumberTitle','off')
subplot(3,1,1), plot(t,h,'linewidth',1.5), grid minor
title('\textbf{Coordenada generalizada $h$ e suas derivadas}','Interpreter', 'latex')
xlabel('$Tempo [s]$','Interpreter', 'latex')
ylabel('$h [mm]$','Interpreter', 'latex')
set(gca,'Fontsize',12,'FontName', 'Times New Roman')
subplot(3,1,2), plot(t,dh,'linewidth',1.5), grid minor
xlabel('$Tempo [s]$','Interpreter', 'latex')
ylabel('$\dot{h} [mm/s]$','Interpreter', 'latex')
set(gca,'Fontsize',12,'FontName', 'Times New Roman')
subplot(3,1,3), plot(t,ddh,'linewidth',1.5), grid minor
xlabel('$Tempo [s]$','Interpreter', 'latex')
ylabel('$\ddot{h} [mm/s^2]$','Interpreter', 'latex')
set(gca,'Fontsize',12,'FontName', 'Times New Roman')