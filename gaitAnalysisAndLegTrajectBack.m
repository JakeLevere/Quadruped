%% Final Project Gait Analysis and Trajecotry Generation - Ethan Lauer
function [alphaDeg,betaDeg,gammaDeg] = gaitAnalysisAndLegTrajectBack(numLegs,coxa,femur,tibia, ...
    theta1Home,theta2Home,S,U,p,beta)
%clc; clear all; close all
%% General and Dimension Constants
% numLegs = 4;
% % Leg Link Lengths (in)
% coxa = 1.5374;
% femur = 3.4638;
% tibia = 5.7477;
% 
% % Platform diamters/Radii (in inches
% top_diam = 5.23;
% bot_diam = top_diam+(2*(coxa+femur));
% topR = top_diam/2;
% botR = bot_diam/2;
% 
% % Home Position angles (for hip joints
% if false % isDeg
%     theta1Home = 53.64;% degrees
%     theta2Home = 54.02;% degrees
% else
%     theta1Home = deg2rad(53.64);% radians
%     theta2Home = deg2rad(54.02);% radians
% end
% 
% [S, U] = SandUVectors(topR, botR, theta1Home, theta2Home); %false

%% Trajectory Constants
%beta = 0.75; % duty factor
% beta = 0.5; % duty factor
yVel_bg = -4; % desired CG velocity of body (in/sec) ( moving forward in y direction wrt grnd)
u_fg = yVel_bg/(1-beta); % ave foot hor forward (y)vel wrt gnd (in/sec)
L = -4; % stride length (in)
u_fb = u_fg-yVel_bg; % ave foot hor forward (y) vel wrt body (in/sec)
T = L/yVel_bg; % cycle time
transferTime=(1-beta)*T;
deltaT = transferTime/4; % 4 different intervals, 5 points
maxFootH = 3; % max foot height (in)
% homeBodH = 5.7477; % max body height (inches) - home position
homeBodH=5;
zVel_bg = 0; % body is not moving vertically wrt gnd
%% Timing constants

% even time interavals between each point
t0 = 0;
t1 = deltaT;
t2 = 2*deltaT;
t3 = 3*deltaT;
t4 = 4*deltaT;
timeMat = [t0,t1,t2,t3,t4];

%% GAIT GENERATION
p(1)=0; % Kinematic phase of leg 1
p(2)=p(1)+1/2; % Kinematic phase of leg 2
p(3)=p(1)+beta; % Kinematic phase of leg 3
p(4)=p(2)+beta; % Kinematic phase of leg 4

j=1;
for j=1:4
    for i=1:4
        if p(i)>=1
            p(i)=p(i)-1;
        end
    end
    j=j+1;
end
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
D = coxa+femur; % distance from hip joint to foot from top view at home position
alphaH=[pi-theta1Home, theta1Home, pi+theta2Home, (2*pi)-theta2Home]; % Home alpha values for each leg
xf_b = [D*cos(alphaH(1)),D*cos(alphaH(2)),D*cos(alphaH(3)),D*cos(alphaH(4))]
yf_b
zf_b
for i =1:numLegs
    % rotation of hip wrt body is z rotation matrix
    for k =1:length(timeMat)
        xf_H(i,k)=[cos(alphaH(i)),-sin(alphaH(i)),0]*[xf_b(i);yf_b(i,k);zf_b(i,k)];
        yf_H(i,k)=[sin(alphaH(i)),cos(alphaH(i)),0]*[xf_b(i);yf_b(i,k);zf_b(i,k)];
        zf_H(i,k)=zf_b(i,k);
    end
end
xf_H
yf_H
zf_H

%% Inverse Kinematics
for i=1:numLegs % for all 4 legs
    for j=1:length(timeMat) % time discrete
        Alpha(i,j)=(atan2(yf_H(i,j),xf_H(i,j)));
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

%% Get the joint positions for each leg during each transfer time
[transTimeJntPosLeg1,transTimeJntPosLeg2,transTimeJntPosLeg3,transTimeJntPosLeg4] = getJntPosInTransTime(Alpha,Beta,Gamma,coxa,femur,tibia, xb_g, yb_g,zb_g);

%% Using the kinematic phase
% for 0.5 duty factor
% this is just one cycle time
time = linspace(0,T,10);
for i=1:numLegs
    yHipB_G(i,1) = S(2,i);
    zHipB_G(i,1) = homeBodH;
    xHipB_G(i,1) =S(1,i); % dont want to move laterally in the x direction??????????????????????????????????
end
% get hip positions for the full phase since you are moving constantly
for i = 1:numLegs
    for t=1:length(time) 
        yHipB_G(i,t+1)=yHipB_G(i,t)+yVel_bg*time(2);% change in time is even
        zHipB_G(i,t+1)=zHipB_G(i,t)+zVel_bg*time(2);
    end
end
for i = 1:length(time)
    t = time(i); % current time
    % store each position in a 12xk array
    % rows 1-3 are hip, 4-6 are knee, 7-9 are ankle, 10-12 is foot pos
    % column for each time
         
    if t>=0 && t<p(2) %leg 2 and 3 moves
        cycleTimeJntPosLeg2(:,i)= transTimeJntPosLeg2(:,i);
        cycleTimeJntPosLeg3(:,i)= transTimeJntPosLeg3(:,i);
        cycleTimeJntPosLeg4(:,i)= transTimeJntPosLeg4(:,1);
        cycleTimeJntPosLeg1(:,i)= transTimeJntPosLeg1(:,1);
    end
    if t>=p(2) && t<=time(end) %leg 1 and 4 moves 
        cycleTimeJntPosLeg1(:,i)= transTimeJntPosLeg1(:,i-(length(time)/2));
        cycleTimeJntPosLeg4(:,i)= transTimeJntPosLeg4(:,i-(length(time)/2));
        cycleTimeJntPosLeg2(:,i)= transTimeJntPosLeg2(:,end);
        cycleTimeJntPosLeg3(:,i)= transTimeJntPosLeg3(:,end);
    end
    
end


%% now for a custom time frame, repeat the cycle

% two cycles
longTime = linspace(0,2*T,20);
for i=1:numLegs
    yHipB_G(i,1) = S(2,i);
    zHipB_G(i,1) = homeBodH;
    xHipB_G(i,1) =S(1,i); % dont want to move laterally in the x direction??????????????????????????????????
end
% get hip positions for the full phase since you are moving constantly
for i = 1:numLegs
    for t=1:length(longTime)-1 
        yHipB_G(i,t+1)=yHipB_G(i,t)+yVel_bg*longTime(2);% change in time is even
        zHipB_G(i,t+1)=zHipB_G(i,t)+zVel_bg*longTime(2);
    end
end
[transTimeJntPosLeg12,transTimeJntPosLeg22,transTimeJntPosLeg32,transTimeJntPosLeg42] = getAllJntPositions(Alpha,Beta,Gamma,coxa,femur,tibia, xHipB_G, yHipB_G,zHipB_G);

for i = 1:length(longTime)
    t = longTime(i); % current time
    % store each position in a 12xk array
    % rows 1-3 are hip, 4-6 are knee, 7-9 are ankle, 10-12 is foot pos
    % column for each time
         
    % assuming only hips are moving together, all other joints are still.
    twoCycleJntPosLeg1(1:3,i) = transTimeJntPosLeg12(1:3,i);
    twoCycleJntPosLeg2(1:3,i) = transTimeJntPosLeg22(1:3,i);
    twoCycleJntPosLeg3(1:3,i) = transTimeJntPosLeg32(1:3,i);
    twoCycleJntPosLeg4(1:3,i) = transTimeJntPosLeg42(1:3,i);
    
    
    if t>=0 && t<p(2) %leg 2 and 3 moves
        twoCycleJntPosLeg2(4:12,i)= transTimeJntPosLeg22(4:12,i);
        twoCycleJntPosLeg3(4:12,i)= transTimeJntPosLeg32(4:12,i);
        twoCycleJntPosLeg4(4:12,i)= transTimeJntPosLeg42(4:12,1);
        twoCycleJntPosLeg1(4:12,i)= transTimeJntPosLeg12(4:12,1);
        k=i;
    end
    if t>=p(2) && t<T %leg 1 and 4 moves 
        twoCycleJntPosLeg1(4:12,i)= transTimeJntPosLeg12(4:12,i);
        twoCycleJntPosLeg4(4:12,i)= transTimeJntPosLeg42(4:12,i);
        twoCycleJntPosLeg2(4:12,i)= transTimeJntPosLeg22(4:12,k);
        twoCycleJntPosLeg3(4:12,i)= transTimeJntPosLeg32(4:12,k);
        r=i;
    end
    if t>=T && t<(T+p(2)) %leg 2 and 3 moves
        twoCycleJntPosLeg2(4:12,i)= transTimeJntPosLeg22(4:12,i);
        twoCycleJntPosLeg3(4:12,i)= transTimeJntPosLeg32(4:12,i);
        twoCycleJntPosLeg4(4:12,i)= transTimeJntPosLeg42(4:12,r);
        twoCycleJntPosLeg1(4:12,i)= transTimeJntPosLeg12(4:12,r);
        k=i;
    end
    if t>=(T+p(2)) && t<=longTime(end) %leg 1 and 4 moves
        twoCycleJntPosLeg1(4:12,i)= transTimeJntPosLeg12(4:12,i);
        twoCycleJntPosLeg4(4:12,i)= transTimeJntPosLeg42(4:12,i);
        twoCycleJntPosLeg2(4:12,i)= transTimeJntPosLeg22(4:12,k);
        twoCycleJntPosLeg3(4:12,i)= transTimeJntPosLeg32(4:12,k);
        r=i;
    end

end


% this is for 0.75 duty factor
%
time = linspace(0,1,16);
for i = 1:length(time)
    t = time(i); % current time

    if t>=0 && t<p(4) %leg 4 moves
        cycleTimeJntPosLeg4(:,i)= transTimeJntPosLeg4(:,i);
        cycleTimeJntPosLeg2(:,i)= transTimeJntPosLeg2(:,1);
        cycleTimeJntPosLeg3(:,i)= transTimeJntPosLeg3(:,1);
        cycleTimeJntPosLeg1(:,i)= transTimeJntPosLeg1(:,1);
    end
    if t>=p(4) && t<p(2) %leg 2 moves
        cycleTimeJntPosLeg2(:,i)= transTimeJntPosLeg2(:,i-4);
        cycleTimeJntPosLeg4(:,i)= transTimeJntPosLeg4(:,end);
        cycleTimeJntPosLeg3(:,i)= transTimeJntPosLeg3(:,1);
        cycleTimeJntPosLeg1(:,i)= transTimeJntPosLeg1(:,1);
    end
    if t>=p(2) && t<p(3) % leg 3 moves
        cycleTimeJntPosLeg3(:,i)= transTimeJntPosLeg3(:,i-8);
        cycleTimeJntPosLeg4(:,i)= transTimeJntPosLeg4(:,end);
        cycleTimeJntPosLeg2(:,i)= transTimeJntPosLeg2(:,end);
        cycleTimeJntPosLeg1(:,i)= transTimeJntPosLeg1(:,1);
    end
    if t>=p(3) && t<=time(end)% leg 1 moves
        cycleTimeJntPosLeg1(:,i)= transTimeJntPosLeg1(:,i-11);
        cycleTimeJntPosLeg2(:,i)= transTimeJntPosLeg2(:,end);
        cycleTimeJntPosLeg4(:,i)= transTimeJntPosLeg4(:,end);
        cycleTimeJntPosLeg3(:,i)= transTimeJntPosLeg3(:,end);
    end
end


%% then get theta dot afterwards as well. ********************
%% animated Stickplot
% animateWalk(timeMat,transTimeJntPosLeg1,transTimeJntPosLeg2,transTimeJntPosLeg3,transTimeJntPosLeg4, ' Stickplot moving legs', 'Stickplot moving legs.gif')

backLims = [-8,8;-16,8;0,8];

%animateWalk(time,cycleTimeJntPosLeg1,cycleTimeJntPosLeg2,cycleTimeJntPosLeg3,cycleTimeJntPosLeg4, 'Walk forward 1 Cycle', 'Walk backward 1 Cycle.gif',backLims)
animateWalk(longTime,twoCycleJntPosLeg1,twoCycleJntPosLeg2,twoCycleJntPosLeg3,twoCycleJntPosLeg4, 'Walk forward 2 Cycle', 'Walk backward 2 Cycle.gif',backLims)

% animateWalk(longTime,transTimeJntPosLeg12,transTimeJntPosLeg22,transTimeJntPosLeg32,transTimeJntPosLeg42, ' Stickplot moving legs', 'Stickplot moving legs kinematic phase 2 cycles v3.gif')

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




