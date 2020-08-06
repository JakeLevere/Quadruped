
clc; clear all; close all

addpath('C:\Users\jaeps\Documents\RBE521 Legged Robotics\Github\Quadruped-master (1)\Quadruped-master\util\current\')
addpath('C:\Users\jaeps\Documents\RBE521 Legged Robotics\Github\Quadruped-master (1)\Quadruped-master\Simulink\')

%% ********************** Parameters ************************%%

%% General and Dimension Constants
numLegs = 4;
% Leg Link Lengths (in)
coxa = 1.5374;
femur = 3.4638;
tibia = 5.7477;

shin_length = tibia;
thigh_length = femur;
hip_length = coxa;

% Platform diamters/Radii (in inches
top_diam = 5.23;
bot_diam = top_diam+(2*(coxa+femur));
topR = top_diam/2;
botR = bot_diam/2;

theta1DegHome = 53.64;% degrees
theta2DegHome = 54.02;% degrees

% Home Position angles (for hip joints
if false % isDeg
    theta1Home = 53.64;% degrees
    theta2Home = 54.02;% degrees
else
    theta1Home = deg2rad(53.64);% radians
    theta2Home = deg2rad(54.02);% radians
end

[S, U] = SandUVectors(topR, botR, theta1Home, theta2Home); %false

%% Trajectory Constants
beta = 0.75; % duty factor
% beta = 0.5; % duty factor


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
p

%% Simulink Parameters

% Simulink Model Parameters
rleg= 0.5;

dist12 = abs(norm((S(:,1)) - (S(:,2))));
dist24 = abs(norm((S(:,2)) - (S(:,4))));
dist34 = abs(norm((S(:,3)) - (S(:,4))));
dist31 = abs(norm((S(:,3)) - (S(:,1))));

% U vector
u1 = U(:,1);
u2 = U(:,2);
u3 = U(:,3);
u4 = U(:,4);

%% Parallel Movement - Simulink 
homePose = [0;0;6;0;0;0];
goalPose = [0;0;7;0;0;0];
timerange = seconds(linspace(0,4,100));
[allAlphai,allBetai,allGammai,supJntPosLeg1,supJntPosLeg2,supJntPosLeg3,supJntPosLeg4] = parallelMoveTrajectV2(homePose, goalPose,0,4,100,0,0);

leg1_alpha = timetable(-[allAlphai(1,:)-53.64].', 'rowTimes', timerange.');
leg2_alpha = timetable(-[allAlphai(2,:)+54.02].', 'rowTimes', timerange.');
leg3_alpha = timetable(-[allAlphai(3,:)+53.64].', 'rowTimes', timerange.');
leg4_alpha = timetable(-[allAlphai(4,:)-54.02].', 'rowTimes', timerange.');

leg1_beta = timetable(allBetai(1,:).', 'rowTimes', timerange.');
leg2_beta = timetable(allBetai(2,:).', 'rowTimes', timerange.');
leg3_beta = timetable(allBetai(3,:).', 'rowTimes', timerange.');
leg4_beta = timetable(allBetai(4,:).', 'rowTimes', timerange.');

leg1_gamma = timetable(allGammai(1,:).'-90, 'rowTimes', timerange.');
leg2_gamma = timetable(allGammai(2,:).'-90, 'rowTimes', timerange.');
leg3_gamma = timetable(allGammai(3,:).'-90, 'rowTimes', timerange.');
leg4_gamma = timetable(allGammai(4,:).'-90, 'rowTimes', timerange.');


simOut = sim('Simulink/Quadruped_Simulink_Parallel', 'ReturnWorkspaceOutputs', 'on');

%% Walk forward - Matlab

[alphaDeg,betaDeg,gammaDeg] = gaitAnalysisAndLegTrajectV1(numLegs,coxa,femur,tibia, ...
    theta1Home,theta2Home,S,U,p,beta);

%% Walk forward - Simulink

[leg1_alpha, leg2_alpha, leg3_alpha, leg4_alpha, ...
    leg1_beta, leg2_beta, leg3_beta, leg4_beta, ...
    leg1_gamma, leg2_gamma, leg3_gamma, leg4_gamma,hip_length,shin_length,thigh_length] = ... 
    Sim_Param_Tables(alphaDeg,betaDeg,gammaDeg,p,coxa,femur,tibia);

simOut = sim('Quadruped_Simulink_Walking','ReturnWorkspaceOutputs','on');

pause(10);

%% Walk backward - Matlab

[alphaDeg,betaDeg,gammaDeg] = gaitAnalysisAndLegTrajectBack(numLegs,coxa,femur,tibia, ...
    theta1Home,theta2Home,S,U,p,beta);

%% Walk backward - Simulink

[leg1_alpha, leg2_alpha, leg3_alpha, leg4_alpha, ...
    leg1_beta, leg2_beta, leg3_beta, leg4_beta, ...
    leg1_gamma, leg2_gamma, leg3_gamma, leg4_gamma,hip_length,shin_length,thigh_length] = ... 
    Sim_Param_Tables(alphaDeg,betaDeg,gammaDeg,p,coxa,femur,tibia);

simOut = sim('Quadruped_Simulink_Walking','ReturnWorkspaceOutputs','on');

pause(10);

%% Walk sideways - Matlab


[alphaDeg,betaDeg,gammaDeg] = gaitAnalysisAndLegTrajectCrab(numLegs,coxa,femur,tibia, ...
    theta1Home,theta2Home,S,U,p,beta);

%% Walk sideways - Simulink
alphaDeg = [-alphaDeg(1,:); -alphaDeg(2,:); -alphaDeg(3,:); -alphaDeg(4,:)]

[leg1_alpha, leg2_alpha, leg3_alpha, leg4_alpha, ...
    leg1_beta, leg2_beta, leg3_beta, leg4_beta, ...
    leg1_gamma, leg2_gamma, leg3_gamma, leg4_gamma,hip_length,shin_length,thigh_length] = ... 
    Sim_Param_Tables(alphaDeg,betaDeg,gammaDeg,p,coxa,femur,tibia);

simOut = sim('Quadruped_Simulink_Walking','ReturnWorkspaceOutputs','on');

pause(10);

%% Turn in a circle - Matlab

%% Turn in a circle - Simulink

