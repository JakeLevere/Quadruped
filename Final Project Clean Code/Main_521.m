%% RBE 521 Final Project Main File
clc; clear all; close all;
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
%% Parallel Mechanism Movements
% These functions generate the list of Alpha, Beta, and Gamma values that
% each leg must move to during the trajectory and also produces a stickplot
% gif of the robot moving.

vertMaxSetPoint = [0;0;8;0;0;0];
vertMinSetPoint = [0;0;4.9;0;0;0];
[vertMaxAlpha,vertMaxBeta,vertMaxGamma] = parallelMoveTrajectV1(homePose, vertMaxSetPoint,isDeg,t0,tf,tstep,v0,vf,'Home to [0;0;8;0;0;0]','Home to vertMaxSetPoint.gif')
[vertMinAlpha,vertMinBeta,vertMinGamma] = parallelMoveTrajectV1(homePose, vertMinSetPoint,isDeg,t0,tf,tstep,v0,vf,'Home to [0;0;4.9;0;0;0]','Home to vertMinSetPoint.gif')

randSetPoint1 = [0;2;5;-5;5;-45];
randSetPoint2 = [1;0.5;4.9;0;0;45];
randSetPoint3 = [2.5;0;6;-15;25;0];
[randPt1Alpha,randPt1Beta,randPt1Gamma] = parallelMoveTrajectV1(homePose, randSetPoint1,isDeg,t0,tf,tstep,v0,vf,'Home to [0;2;5;-5;5;-45]','Home to randSetPoint1.gif')
[randPt2Alpha,randPt2Beta,randPt2Gamma] = parallelMoveTrajectV1(homePose, randSetPoint2,isDeg,t0,tf,tstep,v0,vf,'Home to [1;0.5;4.9;0;0;45]','Home to randSetPoint2.gif')
[randPt3Alpha,randPt3Beta,randPt3Gamma] = parallelMoveTrajectV1(homePose, randSetPoint3,isDeg,t0,tf,tstep,v0,vf,'Home to [2.5;0;6;-15;25;0]','Home to randSetPoint3.gif')


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

% Forward
[Alpha_F,Beta_F,Gamma_F,p_F,tTJntPosLeg1_F,tTJntPosLeg2_F,tTJntPosLeg3_F,tTJntPosLeg4_F] = gaitLegTrajFunFwdBack(beta,yVel_bgF,L_p,homeBodH,maxFootH)
[cycleTimeJntPosLeg1_F,cycleTimeJntPosLeg2_F,cycleTimeJntPosLeg3_F,cycleTimeJntPosLeg4_F,time_F] = cyclePosWithPhase(p_F, tTJntPosLeg1_F,tTJntPosLeg2_F,tTJntPosLeg3_F,tTJntPosLeg4_F);
animateWalk(time_F,cycleTimeJntPosLeg1_F,cycleTimeJntPosLeg2_F,cycleTimeJntPosLeg3_F,cycleTimeJntPosLeg4_F, 'Walk Forward 1 Cycle', 'Walk Forward 1 Cycle.gif',forLims)


% Backward
[Alpha_B,Beta_B,Gamma_B,p_B,tTJntPosLeg1_B,tTJntPosLeg2_B,tTJntPosLeg3_B,tTJntPosLeg4_B] = gaitLegTrajFunFwdBack(beta,yVel_bgB,L_n,homeBodH,maxFootH)
[cycleTimeJntPosLeg1_B,cycleTimeJntPosLeg2_B,cycleTimeJntPosLeg3_B,cycleTimeJntPosLeg4_B,time_B] = cyclePosWithPhase(p_B, tTJntPosLeg4_B,tTJntPosLeg3_B,tTJntPosLeg2_B,tTJntPosLeg1_B);
animateWalk(time_B,cycleTimeJntPosLeg1_B,cycleTimeJntPosLeg2_B,cycleTimeJntPosLeg3_B,cycleTimeJntPosLeg4_B, 'Walk Backward 1 Cycle', 'Walk Backward 1 Cycle.gif',backLims)


% Right
[Alpha_R,Beta_R,Gamma_R,p_R,tTJntPosLeg1_R,tTJntPosLeg2_R,tTJntPosLeg3_R,tTJntPosLeg4_R] = gaitLegTrajFunLeftRight(beta,xVel_bgR,L_p,homeBodH, maxFootH)
[cycleTimeJntPosLeg1_R,cycleTimeJntPosLeg2_R,cycleTimeJntPosLeg3_R,cycleTimeJntPosLeg4_R,time_R] = cyclePosWithPhase(p_R, tTJntPosLeg2_R,tTJntPosLeg4_R,tTJntPosLeg1_R,tTJntPosLeg3_R);
animateWalk(time_R,cycleTimeJntPosLeg1_R,cycleTimeJntPosLeg2_R,cycleTimeJntPosLeg3_R,cycleTimeJntPosLeg4_R, 'Walk Side Right 1 Cycle', 'Walk Side Right 1 Cycle.gif',rightLims)

% Left
[Alpha_L,Beta_L,Gamma_L,p_L,tTJntPosLeg1_L,tTJntPosLeg2_L,tTJntPosLeg3_L,tTJntPosLeg4_L] = gaitLegTrajFunLeftRight(beta,xVel_bgL,L_n,homeBodH, maxFootH)
[cycleTimeJntPosLeg1_L,cycleTimeJntPosLeg2_L,cycleTimeJntPosLeg3_L,cycleTimeJntPosLeg4_L,time_L] = cyclePosWithPhase(p_L, tTJntPosLeg3_L,tTJntPosLeg1_L,tTJntPosLeg4_L,tTJntPosLeg2_L);
animateWalk(time_L,cycleTimeJntPosLeg1_L,cycleTimeJntPosLeg2_L,cycleTimeJntPosLeg3_L,cycleTimeJntPosLeg4_L, 'Walk Side Left 1 Cycle', 'Walk Side left 1 Cycle.gif',leftLims)

% Rotating Left
[Alpha_RL,Beta_RL,Gamma_RL,p_RL,tTJntPosLeg1_RL,tTJntPosLeg2_RL,tTJntPosLeg3_RL,tTJntPosLeg4_RL] = gaitLegTrajFunTurn(beta,angVelZPos,strideRotPos,homeBodH, maxFootH)
[cycleTimeJntPosLeg1_RL,cycleTimeJntPosLeg2_RL,cycleTimeJntPosLeg3_RL,cycleTimeJntPosLeg4_RL,time_RL] = cyclePosWithPhase(p_RL, tTJntPosLeg1_RL,tTJntPosLeg2_RL,tTJntPosLeg3_RL,tTJntPosLeg4_RL);
animateWalk(time_RL,cycleTimeJntPosLeg1_RL,cycleTimeJntPosLeg2_RL,cycleTimeJntPosLeg3_RL,cycleTimeJntPosLeg4_RL, 'Turn Left 1 Cycle', 'Turn Left 1 Cycle.gif',rotLims)

% Rotating Right
[Alpha_RR,Beta_RR,Gamma_RR,p_RR,tTJntPosLeg1_RR,tTJntPosLeg2_RR,tTJntPosLeg3_RR,tTJntPosLeg4_RR] = gaitLegTrajFunTurn(beta,angVelZNeg,strideRotNeg,homeBodH, maxFootH)
[cycleTimeJntPosLeg1_RR,cycleTimeJntPosLeg2_RR,cycleTimeJntPosLeg3_RR,cycleTimeJntPosLeg4_RR,time_RR] = cyclePosWithPhase(p_RR, tTJntPosLeg1_RR,tTJntPosLeg2_RR,tTJntPosLeg3_RR,tTJntPosLeg4_RR);
animateWalk(time_RR,cycleTimeJntPosLeg1_RR,cycleTimeJntPosLeg2_RR,cycleTimeJntPosLeg3_RR,cycleTimeJntPosLeg4_RR, 'Turn Right 1 Cycle', 'Turn Right 1 Cycle.gif',rotLims)


