%% Turning Gait Analysis and Trajecotry Generation - Ethan Lauer
% This function gets the kinematic phase, leg joint angles, and joint
% positions for each leg in the transfer phase when turning.
% The leg trajectory is a sine wave in the z direction and a circle in the
% xy plane
%
% Input: beta - duty factor between 0 and 1 (0.75 used)
%       angVelZ - body  angular velocity about the z axis (rad/sec)
%       strideRot - radians (in) the body rotates in one cycle
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

function [Alpha,Beta,Gamma,p,tTJntPosLeg1,tTJntPosLeg2,tTJntPosLeg3,tTJntPosLeg4] = gaitLegTrajFunTurn(beta,angVelZ,strideRot,constHeight, maxFH)
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
D = coxa+femur; 

% Home Position angles (for hip joints
theta1Home = deg2rad(53.64);% radians
theta2Home = deg2rad(54.02);% radians

[S, ~] = SandUVectors(topR, botR, theta1Home, theta2Home);

%% 
% Trajectory Constants
zWVel_bg = angVelZ; % desired angular velocity (rad/sec)
Rad = strideRot; % stride length (rotation amount in radians)
T = Rad/zWVel_bg; % cycle time
transferTime=(1-beta)*T;
deltaT = transferTime/4; % 4 different intervals, 5 points
maxFootH = maxFH; % max foot height (in)
homeBodH=constHeight;%5.7477;
zVel_bg = 0; % body is not moving vertically wrt gnd
xyVel_bg = zWVel_bg*botR; % linear velocity of body in the xy plane

arcLength = D*Rad; % might have to change botR to what ever the Radius we want to be at

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
% Trajectory for rotation

% go in circle in xy plane and in sine wave up and down
xy0 = 0;
xyf = arcLength; % length it will take for to move x and y position

zv0=0;
zvf=0;
[xya0,xya1,xya2,xya3] = cubicTrajectConsts(t0,t4,xy0,xyf,zv0,zvf);

w = pi/arcLength; % frequency so only looking at positive sine wave

wqSpace2 = linspace(0,Rad,length(timeMat)); % will transform to each leg later with the alpha values
for i=1:length(wqSpace2)
    if i ==1
        w2(i) = wqSpace2(i)/(timeMat(i)+0.01);
    else
        w2(i) = wqSpace2(i)/(timeMat(i));
    end
end
for i = 1:length(timeMat)
    t = timeMat(i);
    xval = D*cos(w2(i)*t);
    yval = D*sin(w2(i)*t);
    dxval=-w2(i)*D*sin(w2(i)*t);
    dyval=w2(i)*D*cos(w2(i)*t);
    [xyval, dxyval] = cubicTrajectEqn(xya0,xya1,xya2,xya3, t);
    zval = maxFootH*sin((w*xyval));
    dzval = maxFootH*w*dxyval*cos(w*xyval);
    posF_g(:,i) =[xval;yval;zval];
    velF_g(:,i) = [dxval;dyval;dzval];
end

%% 
% x y and z position of body wrt ground

% rows are legs, columns are instances in time initialize to home position
for i=1:numLegs
    % position of body wrt ground
    xb_g(i,1) =S(1,i);
    yb_g(i,1) = S(2,i);
    zb_g(i,1) = homeBodH;
    
    % position of foot wrt gnd
    xf_g(i,:) = posF_g(1,:);
    yf_g(i,:) = posF_g(2,:); % put same foot to ground position per leg
    zf_g(i,:) = posF_g(3,:);
end

%% 
% Get Body Position wrt ground during transfer time and then get foot position wrt body
alphaH=[pi-theta1Home, theta1Home, pi+theta2Home, (2*pi)-theta2Home]; % Home alpha values for each leg
%  rows are legs, columns are instances in time
for i = 1:numLegs
    for t=1:length(timeMat) % 5 is because of time dividing to 5 equal extents.
        deltaRad= zWVel_bg*timeMat(t); % the differene in radians
        currRad = deltaRad+alphaH(i);
        % then calculate the posiiton based on those change in radians
        xb_g(i,t+1)=topR*cos(currRad);
        yb_g(i,t+1)=topR*sin(currRad);
        zb_g(i,t+1)=zb_g(i,t)+zVel_bg*deltaT;
    end
    for t=1:length(timeMat)
        xf_b(i,t)=xf_g(i,t)-xb_g(i,t);
        yf_b(i,t)=yf_g(i,t)-yb_g(i,t);
        zf_b(i,t)=zf_g(i,t)-zb_g(i,t);
    end
end

for i =1:numLegs
    % rotation of hip wrt body is z rotation matrix
    for k =1:length(timeMat)
        xf_H(i,k)=[cos(alphaH(i)),-sin(alphaH(i)),0]*[xf_b(i,k);yf_b(i,k);zf_b(i,k)];
        yf_H(i,k)=[sin(alphaH(i)),cos(alphaH(i)),0]*[xf_b(i,k);yf_b(i,k);zf_b(i,k)];
        zf_H(i,k)=zf_b(i,k);
    end
end

%% 
% Inverse Kinematics
[Alpha,Beta,Gamma] = invKinTransfer(xf_H,yf_H,zf_H,timeMat, coxa,femur,tibia);
%% 
% Get joint positions for plotting
[tTJntPosLeg1,tTJntPosLeg2,tTJntPosLeg3,tTJntPosLeg4] = getJntPosInTransTimeTurn(Alpha,Beta,Gamma,coxa,femur,tibia, xb_g, yb_g,zb_g);

%% 
% Plotting
trajFig = figure('Name','Circle Trajectory with sin');
plot3(posF_g(1,:),posF_g(2,:),posF_g(3,:),'k-')
hold on
plot3(xb_g(1,:), yb_g(1,:),zb_g(1,:),'-r')
hold on
plot3(xb_g(2,:), yb_g(2,:),zb_g(2,:),'-g')
hold on
plot3(xb_g(3,:), yb_g(3,:),zb_g(3,:),'-b')
hold on
plot3(xb_g(4,:), yb_g(4,:),zb_g(4,:),'-c')
hold on
legend('Foot','Hip1','Hip2','Hip3','Hip4')
xlim([-8,8]);
ylim([-8,8]);
grid on
xlabel('x pos (in)')
ylabel('y pos (in)')
title('Circle Trajectory with sine wrt gnd')
saveas(trajFig,'Turning Trajectories.png')


foot6Fig = figure('Name','Foot Positions and Velocities wrt Gnd Turning');
subplot(3,2,1)
plot(timeMat,posF_g(1,:),'r-')
grid on
xlabel('time (s)')
ylabel('x pos (in)')
title('X pos vs time wrt gnd')

subplot(3,2,2)
plot(timeMat,velF_g(1,:),'r-')
grid on
xlabel('time (s)')
ylabel('dX vel (in/s)')
title('DX vel vs time wrt gnd')

subplot(3,2,3)
plot(timeMat,posF_g(2,:),'r-')
grid on
xlabel('time (s)')
ylabel('y pos (in)')
title('Y pos vs time wrt gnd')

subplot(3,2,4)
plot(timeMat,velF_g(2,:),'r-')
grid on
xlabel('time (s)')
ylabel('dy vel (in/s)')
title('DY vel vs time wrt gnd')

subplot(3,2,5)
plot(timeMat,posF_g(3,:),'r-')
grid on
xlabel('time (s)')
ylabel('z pos (in)')
title('Z pos vs time wrt gnd')

subplot(3,2,6)
plot(timeMat,velF_g(3,:),'r-')
grid on
xlabel('time (s)')
ylabel('dz vel (in/s)')
title('Dz vel vs time wrt gnd')
saveas(foot6Fig,'Turning Foot Pos and Vel.png')

end