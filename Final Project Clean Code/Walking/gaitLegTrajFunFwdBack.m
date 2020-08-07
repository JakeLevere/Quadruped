%% Forward/Backward Gait Analysis and Trajecotry Generation - Ethan Lauer
% This function gets the kinematic phase, leg joint angles, and joint
% positions for each leg in the transfer phase when moving forward or
% backwards. The leg trajectory is a sine wave.
%
% Input: beta - duty factor between 0 and 1 (0.75 used)
%       yVel - body velocity in the y direction (in/sec) 
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

function [Alpha,Beta,Gamma,p,tTJntPosLeg1,tTJntPosLeg2,tTJntPosLeg3,tTJntPosLeg4,timeMat,alphaVels,betaVels,gammaVels,posF_g,velF_g] = gaitLegTrajFunFwdBack(numLegs,coxa,femur,tibia,S,U,theta1Home,theta2Home,beta,yVel,strideLength,constHeight, maxFH)
%%
% Constants
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
% theta1Home = deg2rad(53.64);% radians
% theta2Home = deg2rad(54.02);% radians
% 
% [S, ~] = SandUVectors(topR, botR, theta1Home, theta2Home);

%% 
% Trajectory Constants

yVel_bg = yVel; % desired CG velocity of body (in/sec) ( moving forward in y direction wrt grnd)
u_fg = yVel_bg/(1-beta); % ave foot hor forward (y)vel wrt gnd (in/sec)
L=strideLength;% stride length (in)
u_fb = u_fg-yVel_bg; % ave foot hor forward (y) vel wrt body (in/sec)
T = L/yVel_bg; % cycle time
transferTime=(1-beta)*T;
deltaT = transferTime/4; % 4 different intervals, 5 points
maxFootH = maxFH; % max foot height (in)
homeBodH = constHeight; % max body height (inches) - home position
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
% Gait Generation
p = gaitGen(beta);
%% 
% Trajectory planning
[posF_g,velF_g] = legTrajSine(L,maxFootH,timeMat);

%% 
% y and z position of body wrt ground

% rows are legs, columns are instances in time initialize to home position
for i=1:numLegs
    yb_g(i,1) = S(2,i);
    zb_g(i,1) = homeBodH;
    xb_g(i,1) =S(1,i); % dont want to move laterally in the x direction
    yf_g(i,:) = posF_g(1,:); % put same foot to ground position per leg
    zf_g(i,:) = posF_g(2,:);
    % foot velocities velocities
    dxf_g(i,:) = zeros(1,length(velF_g));
    dyf_g(i,:) = velF_g(1,:);
    dzf_g(i,:) = velF_g(2,:);
end
%% 
% Get Body Position wrt ground during transfer time and then get foot position wrt body

%  rows are legs, columns are instances in time
for i = 1:numLegs
    for t=1:length(timeMat) % 5 is becaus of time dividing to 5 equal extents.
        yb_g(i,t+1)=yb_g(i,t)+yVel_bg*deltaT;
        zb_g(i,t+1)=zb_g(i,t)+zVel_bg*deltaT;
    end
    for t=1:length(timeMat)
        yf_b(i,t)=yf_g(i,t)-yb_g(i,t);
        zf_b(i,t)=zf_g(i,t)-zb_g(i,t);
        % foot velocities
        dxf_b(t)=dxf_g(t);
        dyf_b(t)=dyf_g(t)-yVel_bg;
        dzf_b(t)=dzf_g(t)-zVel_bg;
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
        % position
        xf_H(i,k)=[cos(alphaH(i)),-sin(alphaH(i)),0]*[xf_b(i);yf_b(i,k);zf_b(i,k)];
        yf_H(i,k)=[sin(alphaH(i)),cos(alphaH(i)),0]*[xf_b(i);yf_b(i,k);zf_b(i,k)];
        zf_H(i,k)=zf_b(i,k);
        
        % velocity
        dxf_H(i,k)=[cos(alphaH(i)),-sin(alphaH(i)),0]*[dxf_b(k);dyf_b(k);dzf_b(k)];
        dyf_H(i,k)=[sin(alphaH(i)),cos(alphaH(i)),0]*[dxf_b(k);dyf_b(k);dzf_b(k)];
        dzf_H(i,k)=dzf_b(k);
        
    end
end
xf_H
yf_H
zf_H

%% 
% Inverse Kinematics
[Alpha,Beta,Gamma] = invKinTransfer(xf_H,yf_H,zf_H,timeMat,coxa,femur,tibia);
%%
% Get Joint Positions in 3d space
[tTJntPosLeg1,tTJntPosLeg2,tTJntPosLeg3,tTJntPosLeg4] = getJntPosInTransTime(Alpha,Beta,Gamma,coxa,femur,tibia, xb_g, yb_g,zb_g);
%%
% Velocities

% Solve jacobian symbolically
syms theta1 theta2 theta3
[~, ~, ~, ~, ~, footPos]=legTransform(theta1, theta2, theta3, coxa, femur, tibia);
qMat = [theta1,theta2, theta3];
for i =1:3 % yz rows - not velocity in the x direction!!
    for k = 1:3 % dof, columns
        Jsym(i,k) = diff(footPos(i),qMat(k)); % 3x3 matrix
    end
end
% use inverse jacobian to find joint velocities
for t=1:length(timeMat)
    for i =1:numLegs
        Jval = eval(subs(Jsym,qMat,[Alpha(i,t),Beta(i,t),Gamma(i,t)]));
        dq(i,t,:) = inv(Jval)*[dxf_H(i,k);dyf_H(i,t);dzf_H(i,t)];
    end
end
% joint velocities at the 6 points
alphaVels = dq(:,:,1)
betaVels = dq(:,:,2)
gammaVels = dq(:,:,3)


%%
% Plotting
%%


end