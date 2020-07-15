%% Final Project Inverse Kinematics test Parallel Movements
clc; clear all; close all;

%%
% See Attached CAD Pictures for Details
% The following dimensions are in inches:

numLegs = 4;
% % Leg Lengths
leg_min = 4.4434; % for prismatic 6.2
leg_max = 9.0552;% for prismatic 10.3

% Leg Link Lengths
coxaL1 =1.5374;
femurL2 = 3.4638;
tibiaL3 = 5.7477;

% Platform diamters/Radii
top_diam = 5.23;
bot_diam = 16.05;
top_diam_par = top_diam+(2*coxaL1);

topR = top_diam/2;
topRPar = top_diam_par/2;
botR = bot_diam/2;

% angles for parallel movement
theta1Deg = 53.64;% degrees
theta1Rad = deg2rad(theta1Deg);% radians
theta2Deg = 54.02;% degrees
theta2Rad = deg2rad(theta2Deg);% radians

%% Setting up S and U Vectors
% Each vector is wrt its own frame (S in {c} and U in {o}
[Si1, Ui1] = SandUVectors(topRPar, botR, theta1Deg, theta2Deg, true)

%% O Vector and Rotation matrix
% for motion straight up and down with no rotation theta1 or theta 3

% in fully crouched position low as it can go
poseDownMin = [0;2;0;0;0;0];
OMin = poseDownMin(1:3);
orientationMin = poseDownMin(4:6);

% in fully extended position high as it can go
poseUpMax = [0;8;0;0;0;0];
OMax = poseUpMax(1:3);
orientationMax = poseUpMax(4:6);

% Rotation matrix
RMin = fullEulerRMat(orientationMin,'ZYZ',true);
RMax = fullEulerRMat(orientationMax,'ZYZ',true);


%% inverse Kinematics - get leg lengths

% L matrix - Leg lengths
% minimum position
for i=1:numLegs
    lmin(:,i)=OMin+RMin*Si1(:,i)-Ui1(:,i);
    Lmin(i) = norm(lmin(:,i),2);
end
Lmin = Lmin

% max position
for i=1:numLegs
    lmax(:,i)=OMax+RMax*Si1(:,i)-Ui1(:,i);
    Lmax(i) = norm(lmax(:,i),2);
end
Lmax = Lmax

%% Getting angles for each joint from law of cosines

% knee angle - in degrees
theta_knee_deg_min_arr =[];
theta_knee_deg_max_arr =[];

for i=1:numLegs
    theta_knee_deg_min = 180-lawOfCos(Lmin(i), tibiaL3,femurL2, 'A',true) % expected 130.39
    theta_knee_deg_max = 180-lawOfCos(Lmax(i), tibiaL3,femurL2, 'A',true) %expected 25.29 from solidworks
    theta_knee_deg_min_arr = [theta_knee_deg_min_arr,theta_knee_deg_min];
    theta_knee_deg_max_arr = [theta_knee_deg_max_arr,theta_knee_deg_max];
end
theta_knee_deg_min_arr
theta_knee_deg_max_arr
% if no rotation theta1 or 3 then you can make a right triangle
% assuming degrees
alpha_min = [];
alpha_max = [];
for i=1:numLegs
    xzHyp_max = sqrt((lmax(1,i)^2)+(lmax(3,i)^2));
    alpha_max = [alpha_max,acosd(xzHyp_max/Lmax(i))];
    xzHyp_min = sqrt((lmin(1,i)^2)+(lmin(3,i)^2));
    alpha_min = [alpha_min,acosd(xzHyp_min/Lmin(i))];
end
alpha_min

alpha_max
theta_thigh_deg_min_arr=[];
theta_thigh_deg_max_arr=[];

for i=1:numLegs
    theta_thigh_deg_min = lawOfCos(Lmin(i), tibiaL3,femurL2, 'B',true);
    theta_thigh_deg_max = lawOfCos(Lmax(i), tibiaL3,femurL2, 'B',true);
    
    theta_thigh_deg_min_arr =[theta_thigh_deg_min_arr,theta_thigh_deg_min - alpha_min(i)];
    theta_thigh_deg_max_arr =[theta_thigh_deg_max_arr,theta_thigh_deg_max - alpha_max(i)];
end
theta_thigh_deg_min_arr % expected 62.66 deg from solidworks
theta_thigh_deg_max_arr % expected -50 deg from solidworks



%% Plotting
figure('Name', 'Stickplot')
t=-pi:0.01:pi;
botCircleX=botR*cos(t);
botCircleY=botR*sin(t);
plot3(botCircleX,botCircleY,zeros(1,numel(botCircleX)),'r-')% circle representing bot platform
hold on
topCircleX=topRPar*cos(t);
topCircleY=topRPar*sin(t);
plot3(topCircleX,topCircleY,OMax(2)*ones(1,numel(topCircleX)), 'r-') % circle representing top platform
hold on
plot3(OMax(1),OMax(3),OMax(2),'*g') % max point
hold on
plot3(0,0,0,'*k') % origin
hold on
for i =1:length(Ui1)
    plot3(Ui1(1,i),Ui1(3,i),Ui1(2,i),'*b');
    hold on
    plot3(Si1(1,i),Si1(3,i),OMax(2),'*b');
    hold on
end
for i=1:length(Ui1)
    plot3([Ui1(1,i);Si1(1,i)],...
        [Ui1(3,i);Si1(3,i)],...
        [Ui1(2,i);OMax(2)],'r--');
    hold on
end
grid on
xlim([-15,15])
ylim([-15,15])
zlim([0,15])
xlabel('X Position (in)');
ylabel('Z Position (in)');
zlabel('Y Position (in)');
title('StickPlot')
