%% Function degree or radian check - by Ethan Lauer
function [sinTheta, cosTheta] = sinCosDegRad(theta,isDeg)
if isDeg
    sinTheta = sind(theta);
    cosTheta = cosd(theta);
else
    sinTheta = sin(theta);
    cosTheta = cos(theta);
end
end