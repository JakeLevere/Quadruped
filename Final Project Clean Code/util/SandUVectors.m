%% S and U Vector SetUp - By Ethan Lauer
% This sets up the S and U Vectors. Legs 1-4 are now based on lecture. 1
% and 3 are on the left side, 2 and 4 are on the right side. 1 and 2 deal
% with theta1, 3 and 4 deal with theta2.
%
% Input: topR - top radius distance from the center to the hip joint (in)
%        botR- bottom radius distance from the center to the foot in the
%              home position (in)
%        theta1 - smallest angle from the X axis to legs 1 and 2 (rad)
%        theta2 - smallest angle from the X axis to legs 3 and 4 (rad)
%
% Output: S - matrix of S vectors for each leg (3x4 - rows are xyz, columns
%             are the legs) - origin of body frame to hip joint
%         U - matrix of U vectors for each leg (3x4 - rows are xyz, columns
%             are the legs) - origin of gnd frame to foot contact
function [S, U] = SandUVectors(topR, botR, theta1, theta2)
% S vector
S1=[-topR*cos(theta1); topR*sin(theta1);0];
S2=[topR*cos(theta1); topR*sin(theta1);0];
S3=[-topR*cos(theta2);-topR*sin(theta2);0];
S4=[topR*cos(theta2); -topR*sin(theta2);0];
S = [S1,S2,S3,S4];

% U vector
U1=[-botR*cos(theta1); botR*sin(theta1);0];
U2=[botR*cos(theta1); botR*sin(theta1);0];
U3=[-botR*cos(theta2);-botR*sin(theta2);0];
U4=[botR*cos(theta2); -botR*sin(theta2);0];
U = [U1,U2,U3,U4];
end