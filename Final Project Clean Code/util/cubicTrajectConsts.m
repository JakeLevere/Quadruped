%% Cubic trajectory Constants - By Ethan Lauer
% This function gets the constant values to be added into a cubic
% trajectory polynomial.
%
% Input: t0 - starting time(s)
%        tf - end time (s)
%        x0 - initial position
%        xf - final position 
%        v0 - initial velocity
%        vf - final velocity
%
% Output: a0,a1,a2,a3 - trajectory constants

function [a0,a1,a2,a3] = cubicTrajectConsts(t0,tf,x0,xf,v0,vf)
b= [x0;v0;xf;vf];

A=[1 t0 t0^2 t0^3;
    0 1 2*t0 3*(t0^2);
    1 tf tf^2 tf^3;
    0 1 2*tf 3*(tf^2)];

Y = inv(A)*b; 
a0 = Y(1);
a1 = Y(2);
a2 = Y(3);
a3 = Y(4);
end