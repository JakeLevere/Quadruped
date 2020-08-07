%% RBE 521 Final Project Main File
clc; clear all; close all;

% CHANGE PATHS TO YOUR WORKSPACE FOLDER 
addpath('C:\Users\jaeps\Documents\RBE521 Legged Robotics\Github\MASTER\Quadruped-master (3)\Quadruped-master\Final Project Clean Code\Walking');
addpath('C:\Users\jaeps\Documents\RBE521 Legged Robotics\Github\MASTER\Quadruped-master (3)\Quadruped-master\Final Project Clean Code\util');
addpath('C:\Users\jaeps\Documents\RBE521 Legged Robotics\Github\MASTER\Quadruped-master (3)\Quadruped-master\Final Project Clean Code\Parallel Mechanism');
addpath('C:\Users\jaeps\Documents\RBE521 Legged Robotics\Github\MASTER\Quadruped-master (3)\Quadruped-master\Final Project Clean Code\Simulink');
addpath('C:\Users\jaeps\Documents\RBE521 Legged Robotics\Github\MASTER\Quadruped-master (3)\Quadruped-master\Final Project Clean Code\Walking\walkingUtil');
addpath('C:\Users\jaeps\Documents\RBE521 Legged Robotics\Github\MASTER\Quadruped-master (3)\Quadruped-master\Final Project Clean Code\Parallel Mechanism\invKinUtil');
addpath('C:\Users\jaeps\Documents\RBE521 Legged Robotics\Github\MASTER\Quadruped-master (3)\Quadruped-master\Final Project Clean Code\Plotting');

%% Parallel Simulation Constants/Variables
homePose = [0;0; 5.7477;0;0;0]; % home position of the robot in the parallel state
% time(seconds)
t0 =0;
tf =1;
tstep =20;
% velocity (in/sec)
v0 = 0;
vf = 0;
% For these simulations, we will be providing the orientation in degrees
isDeg = true;

%% 
% Constants
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
theta1DegHome = 53.64;% degrees
theta2DegHome = 54.02;% degrees
%theta1Home = 53.64;% degrees
%theta2Home = 54.02;% degrees
theta1Home = deg2rad(53.64);% radians
theta2Home = deg2rad(54.02);% radians

[S, U] = SandUVectors(topR, botR, theta1Home, theta2Home);

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

%% Parallel Mechanism Movements
% These functions generate the list of Alpha, Beta, and Gamma values that
% each leg must move to during the trajectory and also produces a stickplot
% gif of the robot moving.

% vertMaxSetPoint = [0;0;8;0;0;0];
% vertMinSetPoint = [0;0;4.9;0;0;0];
% [vertMaxAlpha,vertMaxBeta,vertMaxGamma] = parallelMoveTrajectV1(homePose, vertMaxSetPoint,isDeg,t0,tf,tstep,v0,vf,'Home to [0;0;8;0;0;0]','Home to vertMaxSetPoint.gif')
% [vertMinAlpha,vertMinBeta,vertMinGamma] = parallelMoveTrajectV1(homePose, vertMinSetPoint,isDeg,t0,tf,tstep,v0,vf,'Home to [0;0;4.9;0;0;0]','Home to vertMinSetPoint.gif')
% 
% 
% randSetPoint1 = [0;2;5;-5;5;-45];
% randSetPoint2 = [1;0.5;4.9;0;0;45];
% randSetPoint3 = [2.5;0;6;-15;25;0];
% [randPt1Alpha,randPt1Beta,randPt1Gamma] = parallelMoveTrajectV1(homePose, randSetPoint1,isDeg,t0,tf,tstep,v0,vf,'Home to [0;2;5;-5;5;-45]','Home to randSetPoint1.gif')
% [randPt2Alpha,randPt2Beta,randPt2Gamma] = parallelMoveTrajectV1(homePose, randSetPoint2,isDeg,t0,tf,tstep,v0,vf,'Home to [1;0.5;4.9;0;0;45]','Home to randSetPoint2.gif')
% [randPt3Alpha,randPt3Beta,randPt3Gamma] = parallelMoveTrajectV1(homePose, randSetPoint3,isDeg,t0,tf,tstep,v0,vf,'Home to [2.5;0;6;-15;25;0]','Home to randSetPoint3.gif')
% 

%% Walking Simulation Constants/Variables
beta = 0.75; % duty factor
maxFootH = 3; % max foot height (in)
homeBodH=5; % constant body height

yVel_bgF = 4; % desired CG velocity of body (in/sec) ( moving forward in +y direction wrt grnd)
yVel_bgB = -4; % desired CG velocity of body (in/sec) ( moving backward in -y direction wrt grnd)

xVel_bgR = 4; % desired CG velocity of body (in/sec) ( moving To the right in +x direction wrt grnd)
xVel_bgL = -4; % desired CG velocity of body (in/sec) ( moving To the left in -x direction wrt grnd)


angVelZPos = pi/4; % desired positive angular velocity (rad/sec)
strideRotPos = pi/4; % stride Positive length (rotation amount in radians)

angVelZNeg = -pi/4; % desired negative angular velocity (rad/sec)
strideRotNeg = -pi/4; % stride negative length (rotation amount in radians)

L_p = 4; % positive stride length (in)
L_n = -4; % negative stride length (in)

% axis limits for animation
forLims = [-8,8;-8,16;0,8];
backLims = [-8,8;-16,8;0,8];
rightLims = [-8,16;-8,8;0,8];
leftLims = [-16,8;-8,8;0,8];
rotLims = [-10,10;-10,10;0,8];

%% Walking Movements

% % Walk Forward - Matlab

[Alpha_F,Beta_F,Gamma_F,p_F,tTJntPosLeg1_F,tTJntPosLeg2_F,tTJntPosLeg3_F,tTJntPosLeg4_F,timeMat,alphaVels,betaVels,gammaVels,posF_g,velF_g] = gaitLegTrajFunFwdBack(numLegs,coxa,femur,tibia,S,U,theta1Home,theta2Home,beta,yVel_bgF,L_p,homeBodH,maxFootH)
[cycleTimeJntPosLeg1_F,cycleTimeJntPosLeg2_F,cycleTimeJntPosLeg3_F,cycleTimeJntPosLeg4_F,time_F] = cyclePosWithPhase(p_F, tTJntPosLeg1_F,tTJntPosLeg2_F,tTJntPosLeg3_F,tTJntPosLeg4_F);
animateWalk(time_F,cycleTimeJntPosLeg1_F,cycleTimeJntPosLeg2_F,cycleTimeJntPosLeg3_F,cycleTimeJntPosLeg4_F, 'Walk Forward 1 Cycle', 'Walk Forward 1 Cycle.gif',forLims)
plot_traj(posF_g,timeMat,velF_g,Alpha_F,Beta_F,Gamma_F,alphaVels,betaVels,gammaVels);

Alpha_F = rad2deg(Alpha_F);
Beta_F = rad2deg(Beta_F);
Gamma_F = rad2deg(Gamma_F);

% Walk Forward - Simulink

[leg1_alpha, leg2_alpha, leg3_alpha, leg4_alpha, ...
   leg1_beta, leg2_beta, leg3_beta, leg4_beta, ...
   leg1_gamma, leg2_gamma, leg3_gamma, leg4_gamma,hip_length,shin_length,thigh_length] = ... 
   Sim_Param_Tables(Alpha_F,Beta_F,Gamma_F,p_F,coxa,femur,tibia);

%simOut = sim('Quadruped_Simulink_Parallel','ReturnWorkspaceOutputs','on');
simOut = sim('Quadruped_Simulink_Walking','ReturnWorkspaceOutputs','on');

pause(10);

% % Walk Backward - Matlab

[Alpha_B,Beta_B,Gamma_B,p_B,tTJntPosLeg1_B,tTJntPosLeg2_B,tTJntPosLeg3_B,tTJntPosLeg4_B,timeMat,alphaVels,betaVels,gammaVels,posF_g,velF_g] = gaitLegTrajFunFwdBack(numLegs,coxa,femur,tibia,S,U,theta1Home,theta2Home,beta,yVel_bgB,L_n,homeBodH,maxFootH)
[cycleTimeJntPosLeg1_B,cycleTimeJntPosLeg2_B,cycleTimeJntPosLeg3_B,cycleTimeJntPosLeg4_B,time_B] = cyclePosWithPhase(p_B, tTJntPosLeg4_B,tTJntPosLeg3_B,tTJntPosLeg2_B,tTJntPosLeg1_B);
animateWalk(time_B,cycleTimeJntPosLeg1_B,cycleTimeJntPosLeg2_B,cycleTimeJntPosLeg3_B,cycleTimeJntPosLeg4_B, 'Walk Backward 1 Cycle', 'Walk Backward 1 Cycle.gif',backLims)
plot_traj(posF_g,timeMat,velF_g,Alpha_B,Beta_B,Gamma_B,alphaVels,betaVels,gammaVels);

Alpha_B = rad2deg(Alpha_B);
Beta_B = rad2deg(Beta_B);
Gamma_B = rad2deg(Gamma_B);

% % Walk Backward - Simulink

[leg1_alpha, leg2_alpha, leg3_alpha, leg4_alpha, ...
    leg1_beta, leg2_beta, leg3_beta, leg4_beta, ...
    leg1_gamma, leg2_gamma, leg3_gamma, leg4_gamma,hip_length,shin_length,thigh_length] = ... 
    Sim_Param_Tables(Alpha_B,Beta_B,Gamma_B,p_B,coxa,femur,tibia);

%simOut = sim('Quadruped_Simulink_Parallel','ReturnWorkspaceOutputs','on');
simOut = sim('Quadruped_Simulink_Walking','ReturnWorkspaceOutputs','on');

pause(10);

% % Walk Right - Matlab

[Alpha_R,Beta_R,Gamma_R,p_R,tTJntPosLeg1_R,tTJntPosLeg2_R,tTJntPosLeg3_R,tTJntPosLeg4_R,timeMat,alphaVels,betaVels,gammaVels,posF_g,velF_g] = gaitLegTrajFunLeftRight(numLegs,coxa,femur,tibia,S,U,theta1Home,theta2Home,beta,xVel_bgR,L_p,homeBodH, maxFootH)
[cycleTimeJntPosLeg1_R,cycleTimeJntPosLeg2_R,cycleTimeJntPosLeg3_R,cycleTimeJntPosLeg4_R,time_R] = cyclePosWithPhase(p_R, tTJntPosLeg2_R,tTJntPosLeg4_R,tTJntPosLeg1_R,tTJntPosLeg3_R);
animateWalk(time_R,cycleTimeJntPosLeg1_R,cycleTimeJntPosLeg2_R,cycleTimeJntPosLeg3_R,cycleTimeJntPosLeg4_R, 'Walk Side Right 1 Cycle', 'Walk Side Right 1 Cycle.gif',rightLims)
plot_traj(posF_g,timeMat,velF_g,Alpha_R,Beta_R,Gamma_R,alphaVels,betaVels,gammaVels);

Alpha_R = rad2deg(Alpha_R);
Beta_R = rad2deg(Beta_R);
Gamma_R = rad2deg(Gamma_R);

% % Walk Right - Simulink

[leg1_alpha, leg2_alpha, leg3_alpha, leg4_alpha, ...
    leg1_beta, leg2_beta, leg3_beta, leg4_beta, ...
    leg1_gamma, leg2_gamma, leg3_gamma, leg4_gamma,hip_length,shin_length,thigh_length] = ... 
    Sim_Param_Tables_Sides(Alpha_R,Beta_R,Gamma_R,p_R,coxa,femur,tibia);

%simOut = sim('Quadruped_Simulink_Parallel','ReturnWorkspaceOutputs','on');
simOut = sim('Quadruped_Simulink_Walking','ReturnWorkspaceOutputs','on');

% % Walk Left - Matlab

[Alpha_L,Beta_L,Gamma_L,p_L,tTJntPosLeg1_L,tTJntPosLeg2_L,tTJntPosLeg3_L,tTJntPosLeg4_L,timeMat,alphaVels,betaVels,gammaVels,posF_g,velF_g] = gaitLegTrajFunLeftRight(numLegs,coxa,femur,tibia,S,U,theta1Home,theta2Home,beta,xVel_bgL,L_n,homeBodH, maxFootH)
[cycleTimeJntPosLeg1_L,cycleTimeJntPosLeg2_L,cycleTimeJntPosLeg3_L,cycleTimeJntPosLeg4_L,time_L] = cyclePosWithPhase(p_L, tTJntPosLeg3_L,tTJntPosLeg1_L,tTJntPosLeg4_L,tTJntPosLeg2_L);
animateWalk(time_L,cycleTimeJntPosLeg1_L,cycleTimeJntPosLeg2_L,cycleTimeJntPosLeg3_L,cycleTimeJntPosLeg4_L, 'Walk Side Left 1 Cycle', 'Walk Side left 1 Cycle.gif',leftLims)
plot_traj(posF_g,timeMat,velF_g,Alpha_L,Beta_L,Gamma_L,alphaVels,betaVels,gammaVels);

Alpha_L = rad2deg(Alpha_L);
Beta_L = rad2deg(Beta_L);
Gamma_L = rad2deg(Gamma_L);

% % Walk Left - Simulink

[leg1_alpha, leg2_alpha, leg3_alpha, leg4_alpha, ...
    leg1_beta, leg2_beta, leg3_beta, leg4_beta, ...
    leg1_gamma, leg2_gamma, leg3_gamma, leg4_gamma,hip_length,shin_length,thigh_length] = ... 
    Sim_Param_Tables_Sides(Alpha_L,Beta_L,Gamma_L,p_L,coxa,femur,tibia);

%simOut = sim('Quadruped_Simulink_Parallel','ReturnWorkspaceOutputs','on');
simOut = sim('Quadruped_Simulink_Walking','ReturnWorkspaceOutputs','on');

% % Rotating Left - Matlab

[Alpha_RL,Beta_RL,Gamma_RL,p_RL,tTJntPosLeg1_RL,tTJntPosLeg2_RL,tTJntPosLeg3_RL,tTJntPosLeg4_RL,timeMat,alphaVels,betaVels,gammaVels,posF_g,velF_g] = gaitLegTrajFunTurn(numLegs,coxa,femur,tibia,S,U,theta1Home,theta2Home,botR,topR,beta,angVelZPos,strideRotPos,homeBodH, maxFootH)
[cycleTimeJntPosLeg1_RL,cycleTimeJntPosLeg2_RL,cycleTimeJntPosLeg3_RL,cycleTimeJntPosLeg4_RL,time_RL] = cyclePosWithPhase(p_RL, tTJntPosLeg1_RL,tTJntPosLeg2_RL,tTJntPosLeg3_RL,tTJntPosLeg4_RL);
animateWalk(time_RL,cycleTimeJntPosLeg1_RL,cycleTimeJntPosLeg2_RL,cycleTimeJntPosLeg3_RL,cycleTimeJntPosLeg4_RL, 'Turn Left 1 Cycle', 'Turn Left 1 Cycle.gif',rotLims)
plot_traj(posF_g,timeMat,velF_g,Alpha_RL,Beta_RL,Gamma_RL,alphaVels,betaVels,gammaVels);

Alpha_RL = rad2deg(Alpha_RL);
Beta_RL = rad2deg(Beta_RL);
Gamma_RL = rad2deg(Gamma_RL);

% Rotating Left - Simulink

[leg1_alpha, leg2_alpha, leg3_alpha, leg4_alpha, ...
    leg1_beta, leg2_beta, leg3_beta, leg4_beta, ...
    leg1_gamma, leg2_gamma, leg3_gamma, leg4_gamma,hip_length,shin_length,thigh_length] = ... 
    Sim_Param_Tables_Turn(Alpha_RL,Beta_RL,Gamma_RL,p_RL,coxa,femur,tibia);

%simOut = sim('Quadruped_Simulink_Parallel','ReturnWorkspaceOutputs','on');
simOut = sim('Quadruped_Simulink_Walking','ReturnWorkspaceOutputs','on');

% Rotating Right - Matlab

[Alpha_RR,Beta_RR,Gamma_RR,p_RR,tTJntPosLeg1_RR,tTJntPosLeg2_RR,tTJntPosLeg3_RR,tTJntPosLeg4_RR,timeMat,alphaVels,betaVels,gammaVels,posF_g,velF_g] = gaitLegTrajFunTurn(numLegs,coxa,femur,tibia,S,U,theta1Home,theta2Home,botR,topR,beta,angVelZNeg,strideRotNeg,homeBodH, maxFootH)
[cycleTimeJntPosLeg1_RR,cycleTimeJntPosLeg2_RR,cycleTimeJntPosLeg3_RR,cycleTimeJntPosLeg4_RR,time_RR] = cyclePosWithPhase(p_RR, tTJntPosLeg1_RR,tTJntPosLeg2_RR,tTJntPosLeg3_RR,tTJntPosLeg4_RR);
animateWalk(time_RR,cycleTimeJntPosLeg1_RR,cycleTimeJntPosLeg2_RR,cycleTimeJntPosLeg3_RR,cycleTimeJntPosLeg4_RR, 'Turn Right 1 Cycle', 'Turn Right 1 Cycle.gif',rotLims)
plot_traj(posF_g,timeMat,velF_g,Alpha_RR,Beta_RR,Gamma_RR,alphaVels,betaVels,gammaVels);

Alpha_RR = rad2deg(Alpha_RR);
Beta_RR = rad2deg(Beta_RR);
Gamma_RR = rad2deg(Gamma_RR);

% Rotating Right - Simulink

[leg1_alpha, leg2_alpha, leg3_alpha, leg4_alpha, ...
   leg1_beta, leg2_beta, leg3_beta, leg4_beta, ...
   leg1_gamma, leg2_gamma, leg3_gamma, leg4_gamma,hip_length,shin_length,thigh_length] = ... 
   Sim_Param_Tables_Turn(Alpha_RR,Beta_RR,Gamma_RR,p_RR,coxa,femur,tibia);

%simOut = sim('Quadruped_Simulink_Parallel','ReturnWorkspaceOutputs','on');
simOut = sim('Quadruped_Simulink_Walking','ReturnWorkspaceOutputs','on');
