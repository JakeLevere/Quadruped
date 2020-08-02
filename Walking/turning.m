%% Turning - Ethan Lauer
clc; clear all; close all;


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


[S, UHome] = SandUVectors(topR, botR, theta1Home, theta2Home);

%% Trajectory Constants
beta = 0.75; % duty factor
zAngVel_bg = pi/4; % desired angular velocity (rad/sec)
Rad = pi/4; % stride length (rotation amount in radians)
T = Rad/zAngVel_bg; % cycle time
transferTime=(1-beta)*T;
deltaT = transferTime/4; % 4 different intervals, 5 points
maxFootH = 3; % max foot height (in)
homeBodH=5;
zVel_bg = 0; % body is not moving vertically wrt gnd

arcLength = botR*Rad; % might have to change botR to what ever the Radius we want to be at


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


%% Trajectory for rotation
% trajecotry= go straight up,, then in move in circle from home position of
% parllell mechanism


% for leg 2 since positive in both x and y
x0 = UHome(1,2);
y0 = UHome(2,2);
xy0 = 0;
xyf = arcLength; % length it will take for to move x and y position

zv0=0;
zvf=0;
[xya0,xya1,xya2,xya3] = cubicTrajectConsts(t0,t4,xy0,xyf,zv0,zvf);

w = pi/arcLength; % frequency so only looking at positive sine wave

wqSpace2 = linspace(theta1Home,theta1Home+Rad,5);
for i=1:length(wqSpace2)
    w2(i) = wqSpace2(i)/(timeMat(i));
end
for i = 1:length(timeMat)
    t = timeMat(i);
    if t==0
        xval= x0;
        yval=y0;
        dxval = 0;
        dyval = 0;
    else
        xval = botR*cos(w2(i)*t);
        yval = botR*sin(w2(i)*t);
        dxval=-w2(i)*botR*sin(w2(i)*t);
        dyval=w2(i)*botR*cos(w2(i)*t);
    end
    [xyval, dxyval] = cubicTrajectEqn(xya0,xya1,xya2,xya3, t);
    zval = maxFootH*sin((w*xyval));
    dzval = maxFootH*w*dxyval*cos(w*xyval);
    posF_g(:,i) =[xval;yval;zval];
    velF_g(:,i) = [dxval;dyval;dzval];
 
end

posF_g

% space = linspace(0,pi/4);
figure('Name','Circle Trajectory')
plot3(posF_g(1,:),posF_g(2,:),posF_g(3,:),'r-')
hold on
% plot (cos(space),sin(space))
xlim([-8,8]);
ylim([-8,8]);
grid on
xlabel('x pos (in)')
ylabel('y pos (in)')
title('Circle Trajectory wrt gnd')



figure('Name','X position over time')
plot(timeMat,posF_g(1,:),'r-')
grid on
xlabel('time (s)')
ylabel('x pos (in)')
title('X position over time wrt gnd')


figure('Name','X velocity over time')
plot(timeMat,velF_g(1,:),'r-')
grid on
xlabel('time (s)')
ylabel('dX vel (in/s)')
title('DX vel over time wrt gnd')

figure('Name','Y position over time')
plot(timeMat,posF_g(2,:),'r-')
grid on
xlabel('time (s)')
ylabel('y pos (in)')
title('Y position over time wrt gnd')

figure('Name','Y velocity over time')
plot(timeMat,velF_g(2,:),'r-')
grid on
xlabel('time (s)')
ylabel('dy vel (in/s)')
title('DY vel over time wrt gnd')

figure('Name','z position over time')
plot(timeMat,posF_g(3,:),'r-')
grid on
xlabel('time (s)')
ylabel('z pos (in)')
title('z position over time wrt gnd')

figure('Name','z velocity over time')
plot(timeMat,velF_g(3,:),'r-')
grid on
xlabel('time (s)')
ylabel('dz vel (in/s)')
title('Dz vel over time wrt gnd')


