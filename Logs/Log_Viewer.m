close all

arduObj = ardupilotreader(C:\Users\linky\OneDrive\Documents\Mission Planner\sitl\flightaxis\logs\00000402.BIN)

% t_tr=93932;
% 
% [~, idx] = min(abs(AHR2.TimeUS - t_tr));
% 
% % Retrieve the nearest value
% AHR2_tr = AHR2.Timeus(idx); 


figure
plot(NTUN.TimeUS,NTUN.Dist)

figure
hold on
lat_sp=zeros(length(AHR2.TimeUS),1);
lat_sp(1)=0;
i=2;
while i<length(lat_sp)
    deg_sp=((AHR2.Lat(i)-AHR2.Lat(i-1))^2+(AHR2.Lng(i)-AHR2.Lng(i-1))^2)^0.5;
    r=6378000;
    r_sp=(r/306)*(2*pi*r);
    lat_sp(i)=r_sp*(AHR2.TimeUS(i)-AHR2.TimeUS(i-1));
    i=i+1;
end
plot(AHR2.TimeUS,lat_sp)
xlabel("time")
ylabel("lat vel (m/s)");
title("Lateral Velocity GPS")
hold off
% 
% figure
% ahr2time=(AHR2.TimeUS(1870:end)-AHR2.TimeUS(1870));
% hold on
% plot((ahr2time),AHR2.Alt(1870:end)-206)
% plot(soln.grid.time*10^6,-soln.grid.state(2,:))
% hold off
% xlabel("Time (s*10^-6)")
% ylabel("Alt (m)")
% title("Altitude")
% legend("State","Desired")
% 
% figure
% hold on
% plot((ahr2time),AHR2.Pitch(1870:end))
% plot(soln.grid.time*10^6,rad2deg(soln.grid.state(3,:)))
% hold off
% xlabel("Time (s*10^-6)")
% ylabel("Pitch (deg)")
% title("Pitch")
% legend("State","Desired")
% 
% tecstime=(TECS.TimeUS(745:end)-TECS.TimeUS(745));
% velnorm=zeros(length(soln.grid.time),1);
% i=1;
% while i<length(velnorm)
%     velnorm(i,1)=((soln.grid.state(4,i)^2)+(soln.grid.state(5,i)^2))^0.5;
%     i=i+1;
% end
% 
% figure
% hold on
% plot((tecstime),TECS.sp(745:end))
% plot(soln.grid.time*10^6,velnorm)
% hold off
% xlabel("Time (e-6 S)")
% ylabel("velocity (m/s)")
% title("Velocity Norm")
% legend("State","Desired")
% 
% figure
% i=1;
% veldiff=zeros(length(soln.grid.time),1);
% while i<length(veldiff)
%     veldiff(i)=-velnorm(i)+TECS.sp(745+5*i);
%     i=i+1;
% end
% plot(soln.grid.time,veldiff)
% xlabel("Time (e-6 S)")
% ylabel("Velocity (m/s)")
% title("Velocity Error (state-desired)")
% 
% figure
% hold on
% plot(NTUN.TimeUS(1870:end)-NTUN.TimeUS(1870),NTUN.Dist(1870:end))
% plot(soln.grid.time(:).*10^6,-soln.grid.state(1,:))
% title("lateral distance")
% 
% figure
% hold on
% lat_sp=zeros(length(AHR2.TimeUS),1);
% i=1;
% while i<length(lat_sp)
%     deg_sp=((AHR2.Lat(i)-AHR2.Lat(i-1))^2+(AHR2.Lng(i)-AHR2.Lng(i-1))^2)^0.5;
%     r=6378000;
%     r_sp=(r/306)*(2*pi*r);
%     lat_sp(i)=r_sp*(AHR2.TimeUS(i)-AHR2.TimeUS(i-1));
% end
% plot(AHR2.TimeUS,lat_sp)
