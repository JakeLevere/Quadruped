%% Test File for Inverse Kinematics Final Project
clc; clear all; close all;
%% Test Position Results
% Home Position
[Alphai,Betai,Gammai,fig] = InverseKinematicsParallelWalking([0;0; 5.7477;0;0;0], 'XYZ', true)
saveas(fig,'Parallel Movement Pose 0-0-5_7477-0-0-0','png');

% High Position
[Alphai2,Betai2,Gammai2,fig2] = InverseKinematicsParallelWalking([0;0;8;0;0;0], 'XYZ', true)
saveas(fig2,'Parallel Movement Pose 0-0-8-0-0-0','png');

% Low Position
[Alphai3,Betai3,Gammai3,fig3] = InverseKinematicsParallelWalking([0;0;4.9;0;0;0], 'XYZ', true)
saveas(fig3,'Parallel Movement Pose 0-0-4_9-0-0-0','png');

% Rotation 1 Position
[Alphai4,Betai4,Gammai4,fig4] = InverseKinematicsParallelWalking([0;2;5;-5;5;-45], 'XYZ', true)
saveas(fig4,'Parallel Movement Pose 0-2-5--5-5--45','png');

% Rotation 2 position
[Alphai5,Betai5,Gammai5,fig5] = InverseKinematicsParallelWalking([1;0.5;4.9;0;0;45], 'XYZ', true)
saveas(fig5,'Parallel Movement Pose 1-0_5-4_9-0-0-45','png');

% Rotation 3 Position
[Alphai6,Betai6,Gammai6,fig6] = InverseKinematicsParallelWalking([2.5;0;6;-15;25;0], 'XYZ', true)
saveas(fig6,'Parallel Movement Pose 2_5-0-6--15-25-0','png');
