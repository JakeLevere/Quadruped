%% Cubic trajectory equation - by Ethan Lauer
% This function produces the position and velocity based on the cubic
% polynomial.
% 
% Input: a0,a1,a2,a3 - cubic polynomial constant values
%        t = point in time
%
% Output: x - position for the given time
%         dx - velocity for the given time
function [x, dx] = cubicTrajectEqn(a0,a1,a2,a3, t)
x = a0+a1*t+a2*t^2+a3*t^3;
dx = a1+2*a2*t+3*a3*(t^2);
end