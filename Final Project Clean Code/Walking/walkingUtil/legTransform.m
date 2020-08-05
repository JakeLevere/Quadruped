%% Leg Transform - Ethan Lauer
% This function returns the leg transformation matrices o the positions of
% each joint.
%
% Input: q1 - hip angle (rad)
%           q2 - knee angle (rad)
%           q3 - ankle angle (rad)
%           coxa - length of link connecting hip and knee
%           femur - length of link connecting ankle and knee
%           tibia- length of link connecting ankle and foot
%
% Output: THip_Knee - transformation matrix from the hip to the knee
%         THip_Ankle- transformation matrix from the hip to the ankle
%         THip_Foot- transformation matrix from the hip to the foot
%         kneePos - knee xyz position in 3D wrt the body/hip
%         anklePos - ankle xyz position in 3D wrt the body/hip
%         footPos - foot xyz position in 3D wrt the body/hip

function [THip_Knee, THip_Ankle, THip_Foot, kneePos, anklePos, footPos]=legTransform(q1,q2,q3, coxa, femur, tibia)
THip_Knee=dhMatrix(q1,0,coxa,-pi/2);
TKnee_Ankle=dhMatrix(-q2,0,femur,0);
TAnkle_Foot=dhMatrix(q3,0,tibia,0);
TFoot_Foot = dhMatrix(pi/2,0,0,pi/2);

THip_Ankle = THip_Knee*TKnee_Ankle;
THip_Foot = THip_Knee*TKnee_Ankle*TAnkle_Foot*TFoot_Foot;

kneePos = THip_Knee(1:3,4);
anklePos = THip_Ankle(1:3,4);
footPos = THip_Foot(1:3,4);
end
