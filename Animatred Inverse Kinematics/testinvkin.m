%% Test File for Inverse Kinematics Final Project - by Ethan Lauer
clc; clear all; close all;

%% Constants while testing
homePose = [0;0; 5.7477;0;0;0];
t0 =0;
tf =5;
tstep =20;
v0 = 0;
vf = 0;
isDeg = true;

%% Test Gif Results
[allAlphai1,allBetai1,allGammai1] = parallelMoveTrajectV1(homePose, [0;0;8;0;0;0],isDeg,t0,tf,tstep,v0,vf,'Stickplot Home position to [0;0;8;0;0;0]','Home_to_0-0-8-0-0-0.gif')

[allAlphai2,allBetai2,allGammai2] = parallelMoveTrajectV1(homePose, [0;2;5;-5;5;-45],isDeg,t0,tf,20,v0,vf,'Stickplot Home position to [0;2;5;-5;5;-45]','Home_to_0-2-5--5-5--45.gif')

[allAlphai3,allBetai3,allGammai3] = parallelMoveTrajectV1(homePose, [2.5;0;6;-15;25;0],isDeg,t0,tf,20,v0,vf,'Stickplot Home position to [0;2;5;-5;5;-45]','Home_to_2.5-0-6--15-25-0.gif')


%% Test Position Results
% Home Position
% [Alphai,Betai,Gammai,fig] = InverseKinematicsParallelWalking([0;0; 5.7477;0;0;0], 'XYZ', true)
% saveas(fig,'Parallel Movement Pose 0-0-5_7477-0-0-0','png');
% 
% % High Position
% [Alphai2,Betai2,Gammai2,fig2] = InverseKinematicsParallelWalking([0;0;8;0;0;0], 'XYZ', true)
% saveas(fig2,'Parallel Movement Pose 0-0-8-0-0-0','png');
% 
% % Low Position
% [Alphai3,Betai3,Gammai3,fig3] = InverseKinematicsParallelWalking([0;0;4.9;0;0;0], 'XYZ', true)
% saveas(fig3,'Parallel Movement Pose 0-0-4_9-0-0-0','png');
% 
% % Rotation 1 Position
% [Alphai4,Betai4,Gammai4,fig4] = InverseKinematicsParallelWalking([0;2;5;-5;5;-45], 'XYZ', true)
% saveas(fig4,'Parallel Movement Pose 0-2-5--5-5--45','png');
% 
% % Rotation 2 position
% [Alphai5,Betai5,Gammai5,fig5] = InverseKinematicsParallelWalking([1;0.5;4.9;0;0;45], 'XYZ', true)
% saveas(fig5,'Parallel Movement Pose 1-0_5-4_9-0-0-45','png');
% 
% % Rotation 3 Position
% [Alphai6,Betai6,Gammai6,fig6] = InverseKinematicsParallelWalking([2.5;0;6;-15;25;0], 'XYZ', true)
% saveas(fig6,'Parallel Movement Pose 2_5-0-6--15-25-0','png');
