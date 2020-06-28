%% S and U Vector Function
% This sets up the S and U Vectors.  legs 1-4 like the cartesian coordinate
% frame (1 is upper right, 2 is upper left, etc in ccw motion). Theta 1 is
% in quadrants 1 and 2, (front legs. back legs are theta 2
function [S, U] = SandUVectors(topR, botR, theta1, theta2, isDeg)
if isDeg
    cosTheta1 = cosd(theta1);
    sinTheta1 = sind(theta1);
    cosTheta2 = cosd(theta2);
    sinTheta2 = sind(theta2);
else
    cosTheta1 = cos(theta1);
    sinTheta1 = sin(theta1);
    cosTheta2 = cos(theta2);
    sinTheta2 = sin(theta2); 
end

% S vector
S1=[topR*cosTheta1; 0; -topR*sinTheta1];
S2=[-topR*cosTheta1; 0; -topR*sinTheta1];
S3=[-topR*cosTheta1; 0; topR*sinTheta1];
S4=[topR*cosTheta1; 0; topR*sinTheta1];
S = [S1,S2,S3,S4];

% U vector
U1=[botR*cosTheta2; 0; -botR*sinTheta2];
U2=[-botR*cosTheta2; 0; -botR*sinTheta2];
U3=[-botR*cosTheta2; 0; botR*sinTheta2];
U4=[botR*cosTheta2; 0; botR*sinTheta2];
U = [U1,U2,U3,U4];

end