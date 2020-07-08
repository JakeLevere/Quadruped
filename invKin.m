%% Inverse kinematics Function
% Provide a pose (6x1) matrix, set of S, U vectors - paramters of the
% robot, rotation type( XYZ, ZYZ, etc) and if degrees or radians

function [Lvect,Lmag] = invKin(pose,S,U,rotType,isDeg)
O = pose(1:3);
if isDeg
    R = rotationVectorToMatrix(deg2rad(pose(4:6)));
else
    R = rotationVectorToMatrix(pose(4:6));
end
Lvect = zeros(3,length(S));
Lmag = zeros(1,length(S));
for i=1:length(S)
    Lvect(:,i)=O+R*S(:,i)-U(:,i);
    Lmag(i) = norm(Lvect(:,i),2);
end
end