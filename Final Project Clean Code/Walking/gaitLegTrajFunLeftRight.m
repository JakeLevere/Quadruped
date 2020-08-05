%% Left/Right Gait Analysis and Trajecotry Generation - Ethan Lauer
% This function gets the kinematic phase, leg joint angles, and joint
% positions for each leg in the transfer phase when moving to the left or
% right. The leg trajectory is a sine wave.
%
% Input: beta - duty factor between 0 and 1 (0.75 used)
%       xVel - body velocity in the x direction (in/sec) 
%       strideLength - length (in) the body moves in one cycle
%       constHeight - height of the body during movement
%       maxFH - maximum foot height when moving in transfer phase
%
% Output: Alpha - list of alpha values each leg must move through during
%               transfer phase (4xn rows are legs columns are trajectory setpoints
%         Beta - list of beta values each leg must move through during
%               transfer phase (4xn rows are legs columns are trajectory setpoints
%         Gamma - list of gamma values each leg must move through during
%               transfer phase (4xn rows are legs columns are trajectory setpoints
%         p - kinematic phase (1x4)
%         tTJntPosLeg1 - list of leg 1 joint positions in 3D space during the
%         transfer time (12xn)
%         tTJntPosLeg2- list of leg 2 joint positions in 3D space during the
%         transfer time (12xn)
%         tTJntPosLeg3- list of leg 3 joint positions in 3D space during the
%         transfer time (12xn)
%         tTJntPosLeg4- list of leg 4 joint positions in 3D space during the
%         transfer time (12xn)
function[Alpha,Beta,Gamma,p,tTJntPosLeg1,tTJntPosLeg2,tTJntPosLeg3,tTJntPosLeg4] = gaitLegTrajFunLeftRight(beta,xVel,strideLength,constHeight, maxFH)
%% 
% General and Dimension Constants
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

[S, U] = SandUVectors(topR, botR, theta1Home, theta2Home);

%% 
% Trajectory Constants
xVel_bg = xVel; % desired CG velocity of body (in/sec) ( moving forward in y direction wrt grnd)
u_fg = xVel_bg/(1-beta); % ave foot hor forward (y)vel wrt gnd (in/sec)
L = strideLength; % stride length (in)
u_fb = u_fg-xVel_bg; % ave foot hor forward (y) vel wrt body (in/sec)
T = L/xVel_bg; % cycle time
transferTime=(1-beta)*T;
deltaT = transferTime/4; % 4 different intervals, 5 points
maxFootH = maxFH; % max foot height (in)
homeBodH=constHeight; % max body height (inches) - home position
zVel_bg = 0; % body is not moving vertically wrt gnd

%% 
% Timing constants

% even time interavals between each point
t0 = 0;
t1 = deltaT;
t2 = 2*deltaT;
t3 = 3*deltaT;
t4 = 4*deltaT;
timeMat = [t0,t1,t2,t3,t4];

%% 
% Gait generation
p = gaitGen(beta);
%% 
% Trajectory planning
[posF_g,velF_g] = legTrajSine(L,maxFootH,timeMat);

%% 
% x and z position of body wrt ground

% rows are legs, columns are instances in time initialize to home position
for i=1:numLegs
    yb_g(i,1) = S(2,i);
    zb_g(i,1) = homeBodH;
    xb_g(i,1) =S(1,i); 
    xf_g(i,:) = posF_g(1,:); % put same foot to ground position per leg
    zf_g(i,:) = posF_g(2,:);
end
%% 
% Get Body Position wrt ground during transfer time and then get foot position wrt body

%  rows are legs, columns are instances in time
for i = 1:numLegs
    for t=1:length(timeMat) % 5 is becaus of time dividing to 5 equal extents.
        xb_g(i,t+1)=xb_g(i,t)+xVel_bg*deltaT;
        zb_g(i,t+1)=zb_g(i,t)+zVel_bg*deltaT;
    end
    for t=1:length(timeMat)
        xf_b(i,t)=xf_g(i,t)-xb_g(i,t);
        zf_b(i,t)=zf_g(i,t)-zb_g(i,t);
    end
end
% y position of each foot wrt base in home position
D = coxa+femur; % distance from hip joint to foot from top view at home position
alphaH=[pi-theta1Home, theta1Home, pi+theta2Home, (2*pi)-theta2Home]; % Home alpha values for each leg
yf_b = [D*sin(alphaH(1)),D*sin(alphaH(2)),D*sin(alphaH(3)),D*sin(alphaH(4))]
xf_b
zf_b
for i =1:numLegs
    % rotation of hip wrt body is z rotation matrix
    for k =1:length(timeMat)
        xf_H(i,k)=[cos(alphaH(i)),-sin(alphaH(i)),0]*[xf_b(i,k);yf_b(i);zf_b(i,k)];
        yf_H(i,k)=[sin(alphaH(i)),cos(alphaH(i)),0]*[xf_b(i,k);yf_b(i);zf_b(i,k)];
        zf_H(i,k)=zf_b(i,k);
    end
end
xf_H
yf_H
zf_H

%% 
% Inverse Kinematics

% have y in x positions since left and right for plotting
[Alpha,Beta,Gamma] = invKinTransfer(yf_H,xf_H,zf_H,timeMat,coxa,femur,tibia);

%% 
% Get the joint positions for each leg during each transfer time
[tTJntPosLeg1,tTJntPosLeg2,tTJntPosLeg3,tTJntPosLeg4] = getJntPosInTransTimeCrab(Alpha,Beta,Gamma,coxa,femur,tibia, xb_g, yb_g,zb_g);
%%
% Plotting
trajFig = figure('Name','Foot Trajectory Left and Right');
plot(posF_g(1,:),posF_g(2,:),'k-')
grid on
xlabel('x pos (in)')
ylabel('z pos (in)')
title('Foot Trajectory Left and Right')
saveas(trajFig,'Left and Right Trajectories.png')

foot6Fig = figure('Name','Foot Positions and Velocities wrt Gnd Left and Right');
subplot(2,2,1)
plot(timeMat,posF_g(1,:),'r-')
grid on
xlabel('time (s)')
ylabel('x pos (in)')
title('X pos vs time wrt gnd')

subplot(2,2,2)
plot(timeMat,velF_g(1,:),'r-')
grid on
xlabel('time (s)')
ylabel('dX vel (in/s)')
title('DX vel vs time wrt gnd')

subplot(2,2,3)
plot(timeMat,posF_g(2,:),'r-')
grid on
xlabel('time (s)')
ylabel('Z pos (in)')
title('Z pos vs time wrt gnd')

subplot(2,2,4)
plot(timeMat,velF_g(2,:),'r-')
grid on
xlabel('time (s)')
ylabel('dz vel (in/s)')
title('DZ vel vs time wrt gnd')

saveas(foot6Fig,'Left and Right Foot Pos and Vel.png')

end