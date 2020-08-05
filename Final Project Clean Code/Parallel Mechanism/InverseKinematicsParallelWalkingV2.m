%% Inverse Kinematics for Walking Robot Parallel Movement Version 2 - By Ethan Lauer
% This function calculates the alpha, beta, and gamma values for each leg
% of the walking robot for parallel movement.
%
% Units: inches, inches/sec, seconds, (caluclations in radians)
%
% Input: goalPose - goal position and orientation the robot (6x1 where the
%                   first 3 rows are the xyz position and the last three 
%                   rows are the rotation wx wx wz
%
% Output: Alphai- alpha values for each leg for this pose (1x4) each column
%                 is a leg
%         Betai- beta values for each leg for this pose (1x4) each column
%                 is a leg
%         Gammai - gamma values for each leg for this pose (1x4) each column
%                 is a leg
%         U - U vector for each leg in this home position (3x4) each row is
%             xyz, each column is a leg
%         hipJntPos - positions of the hip joints in this pose (3x4) each row is
%                     xyz, each column is a leg
%         kneeJntPos- positions of the knee joints in this pose (3x4) each row is
%                     xyz, each column is a leg
%         ankleJntPos- positions of the ankle joints in this pose (3x4) each row is
%                     xyz, each column is a leg
%         footJntPos- positions of the foot joints in this pose (3x4) each row is
%                     xyz, each column is a leg
%
% Additional Notes
%
% Min vertical Position with no rotation: [0;0;4.4475;0;0;0]
% Max vertical Position with no rotation: [0;0;8.3579;0;0;0]
% Hip angle min for theta1 legs 1 and 2 = -37.3 degrees
% Hip angle max for theta1 legs 1 and 2 = 35.24 degrees
% Hip angle min for theta2 legs 3 and 4 = -43.29 degrees
% Hip angle max for theta2 legs 3 and 4 = 43.36 degrees
% Leg Lengths
% leg_min = 4.4434; % for prismatic 6.2
% leg_max = 9.0552;% for prismatic 10.3

function [Alphai,Betai,Gammai,U, hipJntPos,kneeJntPos,ankleJntPos,footJntPos] = InverseKinematicsParallelWalkingV2(goalPose)
%%
% Define constants
numLegs = 4;

% Leg Link Lengths
coxaL1 = 1.5374;
femurL2 = 3.4638;
tibiaL3 = 5.7477;

% Platform diamters/Radii
top_diam = 5.23;
bot_diam = top_diam+(2*(coxaL1+femurL2));
topR = top_diam/2;
botR = bot_diam/2;

% Home Position angles (for hip joints
theta1Home = deg2rad(53.64);% radians
theta2Home = deg2rad(54.02);% radians

%%
% Get S and U Vectors
[Si1, U] = SandUVectors(topR, botR, theta1Home, theta2Home);
%%
% Find Li vector (format: rows are x,y,z, colums are legs)
[Li_vect,~] = invKin(goalPose,Si1,U);
%% 
% Get si2 and alpha values
[Si2,alphai] = getSi2Alphai(Si1,coxaL1,Li_vect);
Alphai=alphai;
%%
% Find Li prime vector (format: rows are x,y,z, colums are legs)
[Li_prime_vect,Li_prime_mag] = invKin(goalPose,Si2,U);

%% 
% Get the angles for beta, gamma and all others nececessary angles
[betai,gammai,~,phii,~] = invKinAnglesWalking(Li_prime_vect, Li_prime_mag,Li_vect,coxaL1,femurL2,tibiaL3);
Betai=betai;
Gammai=gammai;
%%
% Get Joint Positions for plotting
[hipJntPos,kneeJntPos,ankleJntPos, footJntPos] = getLegJointPos(Li_vect, Li_prime_vect, U, theta1Home,theta2Home, alphai, betai, phii,femurL2);
end