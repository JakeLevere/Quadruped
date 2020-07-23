%% Final Project Gait Analysis and Trajecotry Generation - Ethan Lauer
clc; clear all; close all
%% General and Dimension Constants
numLegs = 4;
% Leg Link Lengths (in)
coxa = 1.5374;
femur = 3.4638;
tibia = 5.7477;

% Platform diamters/Radii (in inches
top_diam = 5.23;
bot_diam = top_diam+(2*(coxa+femur));
topR = top_diam/2;
botR = bot_diam/2;

% Home Position angles (for hip joints
if false % isDeg
    theta1Home = 53.64;% degrees
    theta2Home = 54.02;% degrees
else
    theta1Home = deg2rad(53.64);% radians
    theta2Home = deg2rad(54.02);% radians    
end

[S, U] = SandUVectors(topR, botR, theta1Home, theta2Home, false);

%% Trajectory Constants
beta = 0.75; % duty factor
yVel_bg = 4; % desired CG velocity of body (in/sec) ( moving forward in y direction wrt grnd)
u_fg = yVel_bg/(1-beta); % ave foot hor forward (y)vel wrt gnd (in/sec)
L = 4; % stride length (in)
u_fb = u_fg-yVel_bg; % ave foot hor forward (y) vel wrt body (in/sec)
T = L/yVel_bg; % cycle time
transferTime=(1-beta)*T;
deltaT = transferTime/4; % 4 different intervals, 5 points
maxFootH = 3; % max foot height (in)
homeBodH = 5.7477; % max body height (inches) - home position
zVel_bg = 0; % body is not moving vertically wrt gnd
%% Timing constants

% even time interavals between each point
t0 = 0;
t1 = deltaT;
t2 = 2*deltaT;
t3 = 3*deltaT;
t4 = 4*deltaT;
timeMat = [t0,t1,t2,t3,t4];


%% Trajectory planning
v0 =0;
vf =0;
y0 =-L/2;
yf = L/2; % should be at stride length at end of traject

[ya0,ya1,ya2,ya3] = cubicTrajectConsts(t0,t4,y0,yf,v0,vf);

% using radians right now*************
w = pi/L; % frequency so only looking at positive sine wave
timeInterval = linspace(t0,t4);
for i = 1:length(timeMat)
    t = timeMat(i);
    % get the y pos and vel values at this time period
    [yval, dyval] = cubicTrajectEqn(ya0,ya1,ya2,ya3, t);
    % row 1 is y position, row 2 is z position
    posF_g(:,i) = [yval;maxFootH*sin((w*yval)+(pi/2))];
    velF_g(:,i) = [dyval;maxFootH*w*dyval*cos(w*yval)];
end

%% y and z position of body wrt ground
% rows are legs, columns are instances in time initialize to home position
for i=1:numLegs
yb_g(i,1) = S(2,i);
zb_g(i,1) = homeBodH;
xb_g(i,1) =S(1,i); % dont want to move laterally in the x direction??????????????????????????????????
yf_g(i,:) = posF_g(1,:); % put same foot to ground position per leg
zf_g(i,:) = posF_g(2,:);
end
%% Get Body Position wrt ground during transfer time and then get foot position wrt body 

%  rows are legs, columns are instances in time
for i = 1:numLegs
    for t=1:length(timeMat) % 5 is becaus of time dividing to 5 equal extents.
        yb_g(i,t+1)=yb_g(i,t)+yVel_bg*deltaT;
        zb_g(i,t+1)=zb_g(i,t)+zVel_bg*deltaT;
    end
    for t=1:length(timeMat)
        yf_b(i,t)=yf_g(i,t)-yb_g(i,t);
        zf_b(i,t)=zf_g(i,t)-zb_g(i,t);
    end
    
end
% x position of each foot wrt base in home position 
D = coxa+femur;
alphaH=[pi-theta1Home, theta1Home, pi+theta2Home, -theta2Home]; % alphaValues for each leg
xf_b = [D*cos(alphaH(1)),D*cos(alphaH(2)),D*cos(alphaH(3)),D*cos(alphaH(4))];

for i =1:numLegs
    % rotation of hip wrt body is z rotation matrix
    for k =1:length(timeMat)
        xf_H(i,k)=[cos(alphaH(i)),-sin(alphaH(i)),0]*[xf_b(i);yf_b(i,k);zf_b(i,k)];
        yf_H(i,k)=[sin(alphaH(i)),cos(alphaH(i)),0]*[xf_b(i);yf_b(i,k);zf_b(i,k)];
        zf_H(i,k)=zf_b(i,k);
    end
end


%% Inverse Kinematics

for i=1:numLegs % for all 4 legs
    for j=1:length(timeMat) % time discrete
        Alpha(i,j)=(atan(yf_H(i,j)/xf_H(i,j)));
        l(i,j)=sqrt(yf_H(i,j)^2+xf_H(i,j)^2);
        d(i,j)=sqrt(zf_H(i,j)^2+(l(i,j)-coxa)^2);
        Beta(i,j)=acos((femur^2+d(i,j)^2-tibia^2)/(2*femur*d(i,j)))-atan(abs(zf_H(i,j))/(l(i,j)-coxa));
        Gamma(i,j)=pi-(acos((femur^2+tibia^2-d(i,j)^2)/(2*femur*tibia)));

    end
end
% put them in degrees
alphaDeg=Alpha*180/pi
betaDeg=Beta*180/pi
gammaDeg=Gamma*180/pi


%% then get theta dot afterwards as well. But want to plot so how to do forward kinematics

% this is just one position for now
figure('Name', ' Stickplot moving legs')
for i =1:numLegs
    % assuming just one position

% use DH paramters to go from hip joint to the foot tip
[~, ~, ~, kneePos_b, anklePos_b, footPos_b]=legTransform(Alpha(i,1),Beta(i,1),Gamma(i,1), coxa, femur, tibia);
    % the joint positions wrt the body. for plotting we want wrt grnd
switch i
    case 1
        kneePos = [-kneePos_b(1)+xb_g(i); 
            kneePos_b(2)+yb_g(i,1);
            kneePos_b(3)+zb_g(i,1)];
        anklePos = [-anklePos_b(1)+xb_g(i);
            anklePos_b(2)+yb_g(i,1);
            -anklePos_b(3)+zb_g(i,1)];
        footPos = [-footPos_b(1)+xb_g(i);
            footPos_b(2)+yb_g(i,1);
            footPos_b(3)+zb_g(i,1)];
    case 2
        kneePos = [kneePos_b(1)+ xb_g(i); 
            -kneePos_b(2)+yb_g(i,1);
            kneePos_b(3)+zb_g(i,1)];
        anklePos = [anklePos_b(1)+ xb_g(i);
            -anklePos_b(2)+yb_g(i,1);
            -anklePos_b(3)+zb_g(i,1)];
        footPos = [footPos_b(1)+ xb_g(i);
            -footPos_b(2)+yb_g(i,1);
            footPos_b(3)+zb_g(i,1)];        
    case 3
        kneePos = [-kneePos_b(1)+xb_g(i); 
            -kneePos_b(2)+yb_g(i,1);
            kneePos_b(3)+zb_g(i,1)];
        anklePos = [-anklePos_b(1)+xb_g(i);
            -anklePos_b(2)+yb_g(i,1);
            -anklePos_b(3)+zb_g(i,1)];
        footPos = [-footPos_b(1)+xb_g(i);
            -footPos_b(2)+yb_g(i,1);
            footPos_b(3)+zb_g(i,1)];
    case 4
        kneePos = [kneePos_b(1)+xb_g(i); 
            kneePos_b(2)+yb_g(i,1);
            kneePos_b(3)+zb_g(i,1)];
        anklePos = [anklePos_b(1)+xb_g(i);
            anklePos_b(2)+yb_g(i,1);
            -anklePos_b(3)+zb_g(i,1)];
        footPos = [footPos_b(1)+xb_g(i);
            footPos_b(2)+yb_g(i,1);
            footPos_b(3)+zb_g(i,1)];
end
    plot3(xb_g(i),yb_g(i,1),zb_g(i,1),'*b'); % hip joints
    hold on
    plot3(kneePos(1),kneePos(2),kneePos(3), 'b*')
    hold on
   plot3(anklePos(1),anklePos(2),anklePos(3), 'b*')
    hold on
    plot3(footPos(1),footPos(2),footPos(3), 'b*')
    hold on
    
    plot3([xb_g(i);kneePos(1)],[yb_g(i,1);kneePos(2)],[zb_g(i,1);kneePos(3)],'-r')
    hold on
    plot3([kneePos(1);anklePos(1)],[kneePos(2);anklePos(2)],[kneePos(3);anklePos(3)],'-r')
    hold on
    plot3([footPos(1);anklePos(1)],[footPos(2);anklePos(2)],[footPos(3);anklePos(3)],'-r')
    hold on
    
end
% plot body rectangle
plot3([xb_g(1);xb_g(2)],[yb_g(1,1);yb_g(2,1)],[zb_g(1,1);zb_g(2,1)],'-r')
hold on
plot3([xb_g(1);xb_g(3)],[yb_g(1,1);yb_g(3,1)],[zb_g(1,1);zb_g(3,1)],'-r')
hold on
plot3([xb_g(4);xb_g(2)],[yb_g(4,1);yb_g(2,1)],[zb_g(4,1);zb_g(2,1)],'-r')
hold on
plot3([xb_g(4);xb_g(3)],[yb_g(4,1);yb_g(3,1)],[zb_g(4,1);zb_g(3,1)],'-r')
hold on
grid on
zlim([0,8])
xlabel('X pos(in)')
ylabel('Y pos(in)')
zlabel('Z pos(in)')
title(' Stickplot moving legs')



%% Plotting 
% figure('Name','Sine Wave Trajectory')
% plot(posF_g(1,:),posF_g(2,:),'r-')
% grid on
% xlabel('y pos (in)')
% ylabel('z pos (in)')
% title('Sine Wave Trajectory wrt gnd')
% 
% figure('Name','Y position over time')
% plot(timeMat,posF_g(1,:),'r-')
% grid on
% xlabel('time (s)')
% ylabel('y pos (in)')
% title('Y position over time wrt gnd')
% 
% 
% figure('Name','Y velocity over time')
% plot(timeMat,velF_g(1,:),'r-')
% grid on
% xlabel('time (s)')
% ylabel('dy vel (in/s)')
% title('DY vel over time wrt gnd')
% 
% figure('Name','z position over time')
% plot(timeMat,posF_g(2,:),'r-')
% grid on
% xlabel('time (s)')
% ylabel('z pos (in)')
% title('z position over time wrt gnd')
% 
% figure('Name','z velocity over time')
% plot(timeMat,velF_g(2,:),'r-')
% grid on
% xlabel('time (s)')
% ylabel('dz vel (in/s)')
% title('Dz vel over time wrt gnd')











