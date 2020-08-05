%% Inverse Kinematics Parallel Mechanism - By Ethan Lauer
% This function calculates the leg lengths and leg vectors for a parallel
% mechanism.
%
% Input: pose - (6x1) matrix first 3 rows are the xyz position and the last
%                three rows are the rotation wx wx wz
%        S - matrix of S vectors for each leg (3x4 - rows are xyz, columns
%             are the legs) - origin of body frame to hip joint
%        U - matrix of U vectors for each leg (3x4 - rows are xyz, columns
%             are the legs) - origin of gnd frame to foot contact
%
% Output: Lvect - 3x4 matrix of the L 3x1 vectors of the foot contact to 
%                 the hip position
%         Lmag - the leg lengths for each leg
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