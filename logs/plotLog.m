clc;
clear;
close all;

% data0 = dlmread('log_20231123-163755.dat',' ');
% data0_o = dlmread('log_20231124-143525_Limacon.dat',' ');

% data1 = dlmread ('log_20231123-164159.dat',' ');
% data1_o = dlmread ('log_20231124-143800_Rose.dat',' ');
data0 = dlmread('log_20240115-094510_Square.dat',' ');
data0_o = dlmread ('log_20240115-094222_Square.dat',' ');

% data2 = dlmread('log_20231123-164445.dat',' ');
% data2_o = dlmread('log_20231124-144220_Eight_Figure.dat',' ');

data1 = dlmread('log_20231124-134739_Circle.dat',' ');
data1_o = dlmread('log_20231127-135141_Circle.dat',' ');

% data4 = dlmread('log_20231124-135653_Moveing_Square.dat',' ');
% data4_o = dlmread('log_20231124-143208_Moveing_Square.dat',' ');

data4 = dlmread('log_20240118-092625_Helix.dat',' ');
data4_o = dlmread('log_20240118-092741_Helix.dat',' ');

smoothFactor = 30
% data0(:,3:5) = data0(:,3:5) + -0.0035+0.003*rand(size(data1(:,3:5)));
% data0(:,3) = smooth(data0(:,3),smoothFactor);
data4(:,3) = smooth(data4(:,3),smoothFactor);
data4(:,4) = smooth(data4(:,4),smoothFactor);
data4(:,5) = smooth(data4(:,5),smoothFactor);
data4_o(:,3) = smooth(data4_o(:,3),smoothFactor);
data4_o(:,4) = smooth(data4_o(:,4),smoothFactor);
data4_o(:,5) = smooth(data4_o(:,5),smoothFactor);

fig = figure();

subplot(1,4,1)
imshow('sim_robot.png')
title('Simulated robot')

subplot(1,4,2)
plot(data0(:,3),data0(:,4),'r-',LineWidth=3,DisplayName='x');
hold on
grid on
box on
plot(data0_o(:,3),data0_o(:,4),'b-',LineWidth=3,DisplayName='x');
plot(data0(:,6),data0(:,7),'k--',LineWidth=2,DisplayName='xd');
% legend
xlabel('X (m)')
ylabel('Y (m)')
title('Square')
xlim([-0.03; 0.03])

subplot(1,4,3)
plot(data1(:,3),data1(:,4),'r-',LineWidth=3,DisplayName='x');
hold on
grid on
box on
plot(data1_o(:,3),data1_o(:,4),'b-',LineWidth=3,DisplayName='x');
plot(data1(:,6),data1(:,7),'k--',LineWidth=2,DisplayName='xd');
% legend
xlim([-0.03; 0.03])
xlabel('X (m)')
ylabel('Y (m)')
title('Circle')

subplot(1,4,4)
plot3(data4(:,3),data4(:,4),data4(:,5),'r-',LineWidth=3,DisplayName='closed-loop');
hold on
plot3(data4_o(:,3),data4_o(:,4),data4_o(:,5),'b-',LineWidth=3,DisplayName='open-loop');
grid on
plot3(data4(:,6),data4(:,7),data4(:,8),'k--',LineWidth=2,DisplayName='reference');
legend
xlabel('X (m)')
ylabel('Y (m)')
zlabel('Z (m)')
title('Helix')

% Set the font properties
fontName = 'Times'; % Specify the font name
fontSize = 14;      % Specify the font size
fontWeight = 'normal';% Specify the font weight

% Find all text objects in the figure
allTextObjects = findall(fig, 'Type', 'text');

% Set the font properties for all text objects
set(allTextObjects, 'FontName', fontName, 'FontSize', fontSize, 'FontWeight', fontWeight);

% Additionally, set the font properties for all axes in the figure
allAxes = findall(fig, 'type', 'axes');
set(allAxes, 'FontName', fontName, 'FontSize', fontSize, 'FontWeight', fontWeight);


