close all
clear

%%% import data %%%l
log_int = ardupilotreader("C:\Users\linky\OneDrive\Documents\Mission Planner\sitl\flightaxis\logs\00000464.BIN");
%436 and 300.0 for motor overide
traj_tf = 10; %total length of traj
log = readMessages(log_int);
Traj = readmatrix("C:\Users\linky\OneDrive\Documents\Mission Planner\sitl\flightaxis\scripts\Y.csv")';
Input = readmatrix("C:\Users\linky\OneDrive\Documents\Mission Planner\sitl\flightaxis\scripts\U.csv")';
loggedOutput = readLoggedOutput(log_int);

%%% generate plots %%%
traj_t=linspace(0,traj_tf,length(Traj(1,:)));
searchStr = "start_transition SET at 299.8 m from VTOL_LAND WP";
idx = find(contains(loggedOutput.Message, searchStr));
log_t=loggedOutput.timestamp(idx);

%%% TECS
tecs = readMessages(log_int, 'MessageNames', {'TECS'});
tecsData = tecs.MsgData{1};
timetecs = tecsData.TimeUS;
[~, idxtecs] = min(abs(timetecs - log_t));
[~, idxtecsend] = min(abs(timetecs - (log_t+seconds(traj_tf))));
timetecs = timetecs(idxtecs:idxtecsend);% 1e6;  % Convert microseconds to seconds
sp = tecsData.sp(idxtecs:idxtecsend);               % Current True Airspeed
dh = tecsData.dh(idxtecs:idxtecsend);


%%% Velocity
i=1;
sp_mag=linspace(0,0,length(Traj));
while i<length(Traj)
    sp_mag(i)=(Traj(4)^2+Traj(5)^2)^0.5;
    i=i+1;
end
figure(1)
hold on
plot (traj_t,sp_mag)
plot (linspace(0,10,length(sp')),sp')
hold off
xlabel("Time (s)")
ylabel("velocity (m/s)")
title("Velocity")
legend("trajectory","Log")


%%% AHR2
ahr2 = readMessages(log_int, 'MessageNames', {'AHR2'});
ahr2Data = ahr2.MsgData{1};
timeahr2 = ahr2Data.TimeUS;
[~, idxahr2] = min(abs(timeahr2 - log_t));
[~, idxahr2end] = min(abs(timeahr2 - (log_t+seconds(traj_tf))));
timeahr2 = timeahr2(idxahr2:idxahr2end);% 1e6;  % Convert microseconds to seconds
pitch = ahr2Data.Pitch(idxahr2:idxahr2end);               % Current True Airspeed
alt = ahr2Data.Alt(idxahr2:idxahr2end);


%%% Pitch
figure(2)
hold on
plot(traj_t,rad2deg(Traj(3,:)))
plot (linspace(0,10,length(pitch')),pitch')
hold off
xlabel("Time (s)")
ylabel("Pitch(deg)")
title("Pitch")
legend("Trajectory","Log")

%%% Alt
figure(3)
hold on
plot(traj_t,(-Traj(2,:)))
plot (linspace(0,10,length(alt')),alt'-206)

hold off
xlabel("Time (s)")
ylabel("Alt (m)")
title("Altitude")
legend("Trajectory","Log")

%%% Alt Rate
figure(4)
hold on
plot (traj_t,-Traj(5,:))
plot (linspace(0,10,length(dh')),dh')
hold off
xlabel("Time (s)")
ylabel("Climb Rate (m/s)")
title("Cimlb Rate")
legend("trajectory","Log")

%%% NTUN
ntun = readMessages(log_int, 'MessageNames', {'NTUN'});
ntunData = ntun.MsgData{1};
timentun= ntunData.TimeUS;
[~, idxntun] = min(abs(timentun - log_t));
[~, idxntunend] = min(abs(timentun - (log_t+seconds(traj_tf))));
timentun = timentun(idxntun:idxntunend);% 1e6;  % Convert microseconds to seconds
dist = ntunData.Dist(idxntun:idxntunend);               % Current True Airspeed


figure(5)
hold on
plot (traj_t,-Traj(1,:))
plot (linspace(0,10,length(dist')),dist')
hold off
xlabel("Time (s)")
ylabel("Lateral Distance (m)")
title("Lateral Dist")
legend("trajectory","Log")


%%% Rate
rate = readMessages(log_int, 'MessageNames', {'RATE'});
rateData = rate.MsgData{1};
timerate= rateData.TimeUS;
[~, idxrate] = min(abs(timerate - log_t));
[~, idxrateend] = min(abs(timerate - (log_t+seconds(traj_tf))));
timerate = timerate(idxrate:idxrateend);% 1e6;  % Convert microseconds to seconds
pr = rateData.P(idxrate:idxrateend);               % Current True Airspeed


figure(6)
hold on
plot (traj_t,rad2deg(Traj(6,:)))
plot (linspace(0,10,length(pr')),pr')
hold off
xlabel("Time (s)")
ylabel("Pitch Rate (deg/s)")
title("Pitch Rate")
legend("trajectory","Log")

rc = readMessages(log_int, 'MessageNames', {'RCOU'});
rcData = rc.MsgData{1};
timerc= rcData.TimeUS;
[~, idxrc] = min(abs(timerc - log_t));
[~, idxrcend] = min(abs(timerc - (log_t+seconds(traj_tf))));
timerc = timerc(idxrc:idxrcend);% 1e6;  % Convert microseconds to seconds
elv = rcData.C1(idxrc:idxrcend); 
motorf1 = rcData.C7(idxrc:idxrcend);
motorf2 = rcData.C5(idxrc:idxrcend);
motorr1 = rcData.C6(idxrc:idxrcend);
motorr2 = rcData.C8(idxrc:idxrcend);

motorf = (motorf1+motorf2)*0.5;
motorr = (motorr1+motorr2)*0.5;

u_cmd_pct_f=[7.099824,54.899822,53.499817,2.899810,50.699806];
u_cmd_pct_r=[56.299824,5.699823,53.499817,52.099812,1.499804];
t_pct=linspace(0,5*0.003,5);
%t_pct=[00:00:53.420417,00:00:53.423978,00:00:53.426013,00:00:53.428047,00:00:53.428047];

ctrl = readMessages(log_int, 'MessageNames', {'CMDL'});
ctrlData = ctrl.MsgData{1};
timectrl= ctrlData.TimeUS;
[~, idxctrl] = min(abs(timectrl - log_t));
[~, idxctrlend] = min(abs(timectrl - (log_t+seconds(traj_tf))));
timectrl = timectrl(idxctrl:idxctrlend);% 1e6;  % Convert microseconds to seconds
r1 = ctrlData.R1(idxctrl:idxctrlend); 
r2 = ctrlData.R2(idxctrl:idxctrlend); 
r3 = ctrlData.R3(idxctrl:idxctrlend); 
r4 = ctrlData.R4(idxctrl:idxctrlend); 
rf_cmd_avg=(r1+r2)/2;
rr_cmd_avg=(r3+r4)/2;

m1 = ctrlData.M1pt(idxctrl:idxctrlend); 
m2 = ctrlData.M2pt(idxctrl:idxctrlend); 
m3 = ctrlData.M3pt(idxctrl:idxctrlend); 
m4 = ctrlData.M4pt(idxctrl:idxctrlend); 
mf_cmd_avg=(m1+m2)/2;
mf_cmd_avg=mf_cmd_avg*10+1000;
mr_cmd_avg=(m3+m4)/2;
mr_cmd_avg=mr_cmd_avg*10+1000;

figure (7)
hold on
plot(traj_t,((Traj(7,:))/10)+1000);
plot(linspace(0,10,length(motorf')),motorf)
% plot(t_pct,((u_cmd_pct_f*10)+1000))
plot(linspace(0,10,length(timectrl)),mf_cmd_avg)
hold off
xlabel("Time (s)")
ylabel("Throttle (%)")
title("Front Motor Average Throttle")
legend("trajectory","Log","Commanded")

figure (8)
hold on
plot(traj_t,((Traj(8,:))/10)+1000);
plot(linspace(0,10,length(motorr')),motorr)
plot(linspace(0,10,length(timectrl)),mr_cmd_avg)
hold off
xlabel("Time (s)")
ylabel("Throttle (%)")
title("Rear Motor Average Throttle")
legend("trajectory","Log","Commanded")

figure (9)
plot(traj_t,Input(1,:))
hold on
plot(linspace(0,10,length(timectrl)),rf_cmd_avg)
hold off
xlabel("Time (s)")
ylabel("Rotor Rate (rpm/s)")
title("Front Rotor Rate")
legend("Trajectory","Commanded")

figure (10)
plot(traj_t,Input(2,:))
hold on
plot(linspace(0,10,length(timectrl)),rr_cmd_avg)
hold off
xlabel("Time (s)")
ylabel("Rotor Rate (rpm/s)")
title("Rear Rotor Rate")
legend("Trajectory","Commanded")