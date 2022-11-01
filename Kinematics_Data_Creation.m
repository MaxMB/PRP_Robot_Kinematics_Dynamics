close, clc;
fID = fopen('h.txt','w'); fprintf(fID,'%f,%f\n',[t;h]); fclose(fID);
fID = fopen('dh.txt','w'); fprintf(fID,'%f,%f\n',[t;dh]); fclose(fID);
fID = fopen('ddh.txt','w'); fprintf(fID,'%f,%f\n',[t;ddh]); fclose(fID);

fID = fopen('theta.txt','w'); fprintf(fID,'%f,%f\n',[t;theta*180/pi]); fclose(fID);
fID = fopen('dtheta.txt','w'); fprintf(fID,'%f,%f\n',[t;dtheta*180/pi]); fclose(fID);
fID = fopen('ddtheta.txt','w'); fprintf(fID,'%f,%f\n',[t;ddtheta*180/pi]); fclose(fID);

fID = fopen('r.txt','w'); fprintf(fID,'%f,%f\n',[t;r]); fclose(fID);
fID = fopen('dr.txt','w'); fprintf(fID,'%f,%f\n',[t;dr]); fclose(fID);
fID = fopen('ddr.txt','w'); fprintf(fID,'%f,%f\n',[t;ddr]); fclose(fID);

%{
i = 1:n;
A = [i; t; r; theta*180/pi; h];
B = [i; t; dr; dtheta*180/pi; dh];
C = [i; t; ddr; ddtheta*180/pi; ddh];

fileID = fopen('Positions.txt','w');
fprintf(fileID,'%6s      %6s       %6s      %6s    %6s\n','i','t[s]','r [mm]','theta[deg]','h[mm]');
fprintf(fileID,'%6.0f %12.4f %12.4f %12.4f %12.4f\n',A);
fclose(fileID);

fileID = fopen('Velocities.txt','w');
fprintf(fileID,'%6s      %6s      %6s   %6s  %6s\n','i','t[s]','dr [mm/s]','dtheta[deg/s]','dh[mm/s]');
fprintf(fileID,'%6.0f %12.4f %12.4f %12.4f %12.4f\n',B);
fclose(fileID);

fileID = fopen('Acelerations.txt','w');
fprintf(fileID,'%6s      %6s      %6s %6s   %6s\n','i','t[s]','ddr [mm/s^2]','ddtheta[deg/s^2]','ddh[mm/s^2]');
fprintf(fileID,'%6.0f %12.4f %12.4f   %12.4f     %12.4f\n',C);
fclose(fileID);
%}