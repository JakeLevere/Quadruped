%% Inverse kinematics Function - By Ethan Lauer
% Provide a pose (6x1) matrix, set of S, U vectors - paramters of the
% robot, rotation type( XYZ, ZYZ, etc) and if degrees or radians

function [Lvect,Lmag] = invKin(pose,S,U)
O = pose(1:3);
    R = rotationVectorToMatrix(pose(4:6));
Lvect = zeros(3,length(S));
Lmag = zeros(1,length(S));
for i=1:length(S)
    Lvect(:,i)=O+R*S(:,i)-U(:,i);
    Lmag(i) = norm(Lvect(:,i),2);
end
end