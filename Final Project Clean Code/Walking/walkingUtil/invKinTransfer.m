%% Inverse Kinematics for Transfer Phase - Ethan Lauer
% This calculates each leg's joint values based on the given positions and
% the leg lengths;
%
% Input: xf_H - x positions of the foot wrt the hip over the time interval
%         yf_H- y positions of the foot wrt the hip over the time interval
%         zf_H- zpositions of the foot wrt the hip over the time interval
%         timeMat - time interval
%         coxa - length of link connecting hip and knee
%         femur- length of link connecting ankle and knee
%         tibia- length of link connecting ankle and foot
%
% Output: Alpha - list of alpha values (rad) for the leg trajectory for each leg
%         Beta - list of beta values (rad) for the leg trajectory for each leg
%         Gamma - list of gamma values (rad) for the leg trajectory for each leg

function [Alpha,Beta,Gamma] = invKinTransfer(xf_H,yf_H,zf_H,timeMat, coxa,femur,tibia)
for i=1:4 % for all 4 legs
    for j=1:length(timeMat) % time 
        Alpha(i,j)=(atan2(yf_H(i,j),xf_H(i,j)));
        l(i,j)=sqrt(yf_H(i,j)^2+xf_H(i,j)^2);
        d(i,j)=sqrt(zf_H(i,j)^2+(l(i,j)-coxa)^2);
        Beta(i,j)=acos((femur^2+d(i,j)^2-tibia^2)/(2*femur*d(i,j)))-atan(abs(zf_H(i,j))/(l(i,j)-coxa));
        Gamma(i,j)=pi-(acos((femur^2+tibia^2-d(i,j)^2)/(2*femur*tibia)));
    end
end
% put them in degrees
alphaDeg=Alpha*180/pi
betaDeg=Beta*180/pi
gammaDeg=Gamma*180/pi
end