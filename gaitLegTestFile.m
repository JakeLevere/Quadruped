%% gait analy test file
clc; clear all; close all;
%% Constants
beta = 0.75; % duty factor
yVel_bg = 4; % desired CG velocity of body (in/sec) ( moving forward in y direction wrt grnd)
L = 4; % stride length (in)
maxFootH = 3; % max foot height (in)
homeBodH=5;

t0 =0;
tf =(3*(L/yVel_bg))/4; % only want the time during the support phase
tstep =5*4; % defined in gaitLegTrajectFunction
v0 = 0;
vf = 0;

%% return joint angles and  positions of legs in the transfer phase
[AlphaT,BetaT,GammaT,p,tTJntPosLeg1,tTJntPosLeg2,tTJntPosLeg3,tTJntPosLeg4] = gaitLegTrajectFunction(beta,yVel_bg,L,homeBodH,maxFootH)
%% returns joint angles posiitons of leg in support phase
[AlphaS,BetaS,GammaS,supJntPosLeg1,supJntPosLeg2,supJntPosLeg3,supJntPosLeg4] = parallelMoveTrajectV2([0;0;5;0;0;0], [0;1;5;0;0;0],t0,tf,tstep,v0,vf)
%% plotting
time = linspace(0,1,16);
for i = 1:length(time)
    t = time(i); % current time

    if t>=0 && t<p(4) %leg 4 moves
        cycleTimeJntPosLeg4(:,i)= tTJntPosLeg4(:,i);
        cycleTimeJntPosLeg2(:,i)= supJntPosLeg2(:,i);
        cycleTimeJntPosLeg3(:,i)= supJntPosLeg3(:,i);
        cycleTimeJntPosLeg1(:,i)= supJntPosLeg1(:,i);
    end
    if t>p(4) && t<p(2) %leg 2 moves
        cycleTimeJntPosLeg2(:,i)= tTJntPosLeg2(:,i-4);
        cycleTimeJntPosLeg4(:,i)= supJntPosLeg4(:,i);
        cycleTimeJntPosLeg3(:,i)= supJntPosLeg3(:,i);
        cycleTimeJntPosLeg1(:,i)= supJntPosLeg1(:,i);
    end
    if t>p(2) && t<p(3) % leg 3 moves
        cycleTimeJntPosLeg3(:,i)= tTJntPosLeg3(:,i-8);
        cycleTimeJntPosLeg4(:,i)= supJntPosLeg4(:,i);
        cycleTimeJntPosLeg2(:,i)= supJntPosLeg2(:,i);
        cycleTimeJntPosLeg1(:,i)= supJntPosLeg1(:,i);
    end
    if t>p(3) && t<=time(end)% leg 1 moves
        cycleTimeJntPosLeg1(:,i)= tTJntPosLeg1(:,i-11);
        cycleTimeJntPosLeg2(:,i)= supJntPosLeg2(:,i);
        cycleTimeJntPosLeg4(:,i)= supJntPosLeg4(:,i);
        cycleTimeJntPosLeg3(:,i)= supJntPosLeg3(:,i);
    end
end

forLims = [-8,8;-8,16;0,8];

animateWalk(time,cycleTimeJntPosLeg1,cycleTimeJntPosLeg2,cycleTimeJntPosLeg3,cycleTimeJntPosLeg4, 'Walk forward With Parallel', 'Walk forward with Parallel.gif',forLims)

