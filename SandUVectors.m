%% S and U Vector Function
% This sets up the S and U Vectors. Legs 1-4 are now based on lecture. 1
% and 3 are on the left side, 2 and 4 are on the right side. 1 and 2 deal
% with theta1, 3 and 4 deal with theta2.
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
S1=[-topR*cosTheta1; topR*sinTheta1;0];
S2=[topR*cosTheta1; topR*sinTheta1;0];
S3=[-topR*cosTheta2;-topR*sinTheta2;0];
S4=[topR*cosTheta2; -topR*sinTheta2;0];
S = [S1,S2,S3,S4];

% U vector
U1=[-botR*cosTheta1; botR*sinTheta1;0];
U2=[botR*cosTheta1; botR*sinTheta1;0];
U3=[-botR*cosTheta2;-botR*sinTheta2;0];
U4=[botR*cosTheta2; -botR*sinTheta2;0];
U = [U1,U2,U3,U4];

end