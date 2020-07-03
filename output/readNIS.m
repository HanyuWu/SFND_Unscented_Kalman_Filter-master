T = readtable('laser_NIS.csv');
laserNIS = table2array(T)
plot(laserNIS(:,1),'LineWidth',2);
hold on;
plot(laserNIS(:,2),'LineWidth',2);
plot(laserNIS(:,3),'LineWidth',2);
title('NIS for Laser Measurements');
size = length(laserNIS(:,2));
x = linspace(1,size,size);
y = ones(1,size)*5.99;
line(x,y,'LineStyle','--');
legend("Car_1","Car_2","Car_3",'95%');

figure()
T2 = readtable('radar_NIS.csv');
radarNIS = table2array(T)
plot(radarNIS(:,1),'LineWidth',2);
hold on;
plot(radarNIS(:,2),'LineWidth',2);
plot(radarNIS(:,3),'LineWidth',2);
title('NIS for Radar Measurements');
size = length(radarNIS(:,2));
x = linspace(1,size,size);
y = ones(1,size)*7.82;
line(x,y,'LineStyle','--');
legend("Car_1","Car_2","Car_3",'95%');
