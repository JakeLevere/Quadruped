%% Inverse Kinematics for walking robot parallel movement
% Given: goal pose as a 6x1 matrix, rotation type ('XYZ') and boolean 
% if using degrees or radians
% Result: Alpha, Beta, and Gama values for each leg and the figure to save

%% Additional Notes
% Min vertical Position with no rotation: [0;0;4.4475;0;0;0]
% Max vertical Position with no rotation: [0;0;8.3579;0;0;0]
% Hip angle min for theta1 legs 1 and 2 = -37.3 degrees
% Hip angle max for theta1 legs 1 and 2 = 35.24 degrees
% Hip angle min for theta2 legs 3 and 4 = -43.29 degrees
% Hip angle max for theta2 legs 3 and 4 = 43.36 degrees
% Leg Lengths
% leg_min = 4.4434; % for prismatic 6.2
% leg_max = 9.0552;% for prismatic 10.3

function [Alphai,Betai,Gammai,U, hipJntPos,kneeJntPos,ankleJntPos] = InverseKinematicsParallelWalkingV2(goalPose, rotType, isDeg)
%% define constants
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
if isDeg
    theta1Home = 53.64;% degrees
    theta2Home = 54.02;% degrees
else
    theta1Home = deg2rad(53.64);% radians
    theta2Home = deg2rad(54.02);% radians    
end

%% get S and U Vectors
[Si1, U] = SandUVectors(topR, botR, theta1Home, theta2Home, isDeg);

%% find Li vector (format: rows are x,y,z, colums are legs)
[Li_vect,~] = invKin(goalPose,Si1,U,rotType,isDeg);

%% Get si2 and alpha values
[Si2,alphai] = getSi2Alphai(Si1,coxaL1,Li_vect, isDeg);
Alphai=alphai;
%% find Li prime vector (format: rows are x,y,z, colums are legs)
[Li_prime_vect,Li_prime_mag] = invKin(goalPose,Si2,U,rotType,isDeg);

%% get the angles for beta, gamma and all others
[betai,gammai,~,phii,~] = invKinAnglesWalking(Li_prime_vect, Li_prime_mag,Li_vect,coxaL1,femurL2,tibiaL3, isDeg);
Betai=betai;
Gammai=gammai;
%% get Joint Positions for plotting
[hipJntPos,kneeJntPos,ankleJntPos] = getLegJointPos(Li_vect, Li_prime_vect, U, theta1Home,theta2Home, alphai, betai, phii,femurL2, isDeg);

end