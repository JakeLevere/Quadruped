%% leg Transform
% this function returns the leg transformation matrices for each joint and
% the positions of each joint as well using radians

function [THip_Knee, THip_Ankle, THip_Foot, kneePos, anklePos, footPos]=legTransform(q1,q2,q3, coxa, femur, tibia)
THip_Knee=dhMatrix(q1,0,coxa,-pi/2);
TKnee_Ankle=dhMatrix(q2,0,femur,0);
TAnkle_Foot=dhMatrix(q3,0,tibia,0);
TFoot_Foot = dhMatrix(pi/2,0,0,pi/2);

THip_Ankle = THip_Knee*TKnee_Ankle;
THip_Foot = THip_Knee*TKnee_Ankle*TAnkle_Foot*TFoot_Foot;

kneePos = THip_Knee(1:3,4);
anklePos = THip_Ankle(1:3,4);
footPos = THip_Foot(1:3,4);
end
