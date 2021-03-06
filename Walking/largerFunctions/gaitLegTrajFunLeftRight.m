%% Final Project Gait Analysis and Trajecotry Generation  Crab- Ethan Lauer
function[] = gaitLegTrajFunLeftRight(beta,xVel,strideLength,constHeight, maxFH)
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

[S, U] = SandUVectors(topR, botR, theta1Home, theta2Home);

%% Trajectory Constants
% beta = 0.5; % duty factor
xVel_bg = xVel; % desired CG velocity of body (in/sec) ( moving forward in y direction wrt grnd)
u_fg = xVel_bg/(1-beta); % ave foot hor forward (y)vel wrt gnd (in/sec)
L = strideLength; % stride length (in)
u_fb = u_fg-xVel_bg; % ave foot hor forward (y) vel wrt body (in/sec)
T = L/xVel_bg; % cycle time
transferTime=(1-beta)*T;
deltaT = transferTime/4; % 4 different intervals, 5 points
maxFootH = maxFH; % max foot height (in)
% homeBodH = 5.7477; % max body height (inches) - home position
homeBodH=constHeight;
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
[posF_g,velF_g] = legTrajSine(L,maxFootH,timeMat)


%% y and z position of body wrt ground
% rows are legs, columns are instances in time initialize to home position
for i=1:numLegs
    yb_g(i,1) = S(2,i);
    zb_g(i,1) = homeBodH;
    xb_g(i,1) =S(1,i); % dont want to move laterally in the x direction??????????????????????????????????
    xf_g(i,:) = posF_g(1,:); % put same foot to ground position per leg
    zf_g(i,:) = posF_g(2,:);
end
%% Get Body Position wrt ground during transfer time and then get foot position wrt body

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

%% Inverse Kinematics

% have y in x positions since left and right for plotting
[Alpha,Beta,Gamma] = invKinTransfer(yf_H,xf_H,zf_H,timeMat,coxa,femur,tibia)

% put them in degrees
alphaDeg=Alpha*180/pi
betaDeg=Beta*180/pi
gammaDeg=Gamma*180/pi

%% Get the joint positions for each leg during each transfer time
[transTimeJntPosLeg1,transTimeJntPosLeg2,transTimeJntPosLeg3,transTimeJntPosLeg4] = getJntPosInTransTimeCrab(Alpha,Beta,Gamma,coxa,femur,tibia, xb_g, yb_g,zb_g);

%% Using the kinematic phase
% for 0.5 duty factor
% this is just one cycle time
time = linspace(0,T,10);
for i=1:numLegs
    xHipB_G(i,1) =S(1,i);
    yHipB_G(i,1) = S(2,i);
    zHipB_G(i,1) = homeBodH;
end
% get hip positions for the full phase since you are moving constantly
for i = 1:numLegs
    for t=1:length(time)
        xHipB_G(i,t+1)=xHipB_G(i,t)+xVel_bg*time(2);% change in time is even
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
        xHipB_G(i,t+1)=xHipB_G(i,t)+xVel_bg*longTime(2);% change in time is even
        zHipB_G(i,t+1)=zHipB_G(i,t)+zVel_bg*longTime(2);
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
    if t>p(4) && t<p(2) %leg 2 moves
        cycleTimeJntPosLeg2(:,i)= transTimeJntPosLeg2(:,i-4);
        cycleTimeJntPosLeg4(:,i)= transTimeJntPosLeg4(:,end);
        cycleTimeJntPosLeg3(:,i)= transTimeJntPosLeg3(:,1);
        cycleTimeJntPosLeg1(:,i)= transTimeJntPosLeg1(:,1);
    end
    if t>p(2) && t<p(3) % leg 3 moves
        cycleTimeJntPosLeg3(:,i)= transTimeJntPosLeg3(:,i-8);
        cycleTimeJntPosLeg4(:,i)= transTimeJntPosLeg4(:,end);
        cycleTimeJntPosLeg2(:,i)= transTimeJntPosLeg2(:,end);
        cycleTimeJntPosLeg1(:,i)= transTimeJntPosLeg1(:,1);
    end
    if t>p(3) && t<=time(end)% leg 1 moves
        cycleTimeJntPosLeg1(:,i)= transTimeJntPosLeg1(:,i-11);
        cycleTimeJntPosLeg2(:,i)= transTimeJntPosLeg2(:,end);
        cycleTimeJntPosLeg4(:,i)= transTimeJntPosLeg4(:,end);
        cycleTimeJntPosLeg3(:,i)= transTimeJntPosLeg3(:,end);
    end
end


%% then get theta dot afterwards as well. ********************
%% animated Stickplot
sideLims = [-8,8;-8,8;0,8];

animateWalk(time,cycleTimeJntPosLeg1,cycleTimeJntPosLeg2,cycleTimeJntPosLeg3,cycleTimeJntPosLeg4, 'Walk side left 1 Cycle', 'Walk side 1 Cycle.gif',sideLims)


end