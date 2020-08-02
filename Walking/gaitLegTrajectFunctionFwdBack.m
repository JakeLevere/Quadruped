%% Final Project Gait Analysis and Trajecotry Generation - Ethan Lauer
function [Alpha,Beta,Gamma,p,tTJntPosLeg1,tTJntPosLeg2,tTJntPosLeg3,tTJntPosLeg4] = gaitLegTrajectFunctionFwdBack(beta,yVel,strideLength,constHeight, maxFH)
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
theta1Home = deg2rad(53.64);% radians
theta2Home = deg2rad(54.02);% radians


[S, ~] = SandUVectors(topR, botR, theta1Home, theta2Home);

%% Trajectory Constants
% beta = 0.75; % duty factor
% beta = 0.5; % duty factor
yVel_bg = yVel;
% yVel_bg = 4; % desired CG velocity of body (in/sec) ( moving forward in y direction wrt grnd)
u_fg = yVel_bg/(1-beta); % ave foot hor forward (y)vel wrt gnd (in/sec)
L=strideLength;
% L = 4; % stride length (in)
u_fb = u_fg-yVel_bg; % ave foot hor forward (y) vel wrt body (in/sec)
T = L/yVel_bg; % cycle time
transferTime=(1-beta)*T;
deltaT = transferTime/4; % 4 different intervals, 5 points
maxFootH = maxFH
% maxFootH = 3; % max foot height (in)
% homeBodH = 5.7477; % max body height (inches) - home position
homeBodH = constHeight;
% homeBodH=5;
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
p = gaitGen(beta)
%% Trajectory planning
v0 =0;
vf =0;
y0 =-L/2;
yf = L/2; % should be at stride length at end of traject

[ya0,ya1,ya2,ya3] = cubicTrajectConsts(t0,t4,y0,yf,v0,vf);

% using radians right now*************
w = pi/L; % frequency so only looking at positive sine wave
% timeInterval = linspace(t0,t4);
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

[tTJntPosLeg1,tTJntPosLeg2,tTJntPosLeg3,tTJntPosLeg4] = getJntPosInTransTime(Alpha,Beta,Gamma,coxa,femur,tibia, xb_g, yb_g,zb_g);

end