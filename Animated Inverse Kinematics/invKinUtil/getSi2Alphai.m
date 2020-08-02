%% Get Si2 Function - By Ethan Lauer
% this function gets the Si2 function for walking robot
% this takes in RADIANS
% L1 is the length of the coxa
% Livect is the Li vector calculated from the invkin just from the parallel
% robotics
% returns the list of si2 vectors for each leg and the alpha matrix for all
% the legs

function [si2,alphai] = getSi2Alphai(si1,coxa,LiVect)
% Home angles
theta1DegHome = 53.64;% degrees
theta1RadHome = deg2rad(theta1DegHome);% radians
theta2DegHome = 54.02;% degrees
theta2RadHome = deg2rad(theta2DegHome);% radians

% Alpha limits
% theta1 legs 1 and 2 = -37.3 degrees -> 35.24deg
% theta2 legs 3 and 4 = -43.29 degrees -> 43.36 degrees
alpha_leg1_2_lowLimDeg = -37.3;
alpha_leg1_2_upLimDeg = 35.24;
alpha_leg3_4_lowLimDeg = -43.29;
alpha_leg3_4_upLimDeg = 43.36;

alpha_leg1_2_lowLimRad = deg2rad(alpha_leg1_2_lowLimDeg);
alpha_leg1_2_upLimRad = deg2rad(alpha_leg1_2_upLimDeg);
alpha_leg3_4_lowLimRad = deg2rad(alpha_leg3_4_lowLimDeg);
alpha_leg3_4_upLimRad = deg2rad(alpha_leg3_4_upLimDeg);


si2 = zeros(3,length(si1));
Alphai = zeros(1,length(si1));
for i =1:length(si1)
    
    Alphai(i) = atan(LiVect(2,i)/LiVect(1,i));
    si2(:,i) = [si1(1,i)+((-1)^i)*coxa*cos(Alphai(i));
        si1(2,i)+((-1)^i)*coxa*sin(Alphai(i));
        si1(3,i)];
end

alphai(1) = round(Alphai(1)+theta1RadHome,6);
alphai(2) = round(Alphai(2)-theta1RadHome,6);
alphai(3) = round(Alphai(3)-theta2RadHome,6);
alphai(4) = round(Alphai(4)+theta2RadHome,6);
for i=1:4
    if i==1||i==2
        if alphai(i)<alpha_leg1_2_lowLimRad || alphai(i)>alpha_leg1_2_upLimRad
            error('Alpha angle for legs 1 or 2 is out of range')
        end
    else
        if alphai(i)<alpha_leg3_4_lowLimRad || alphai(i)>alpha_leg3_4_upLimRad
            error('Alpha angle for legs 3 or 4 is out of range')
        end
    end
end

end