%% Cubic trajectory function in task space - By Ethan Lauer
% this gets the constant values that then need to be added into a cubic
% trajectory function for each x y and z, and omegas. 
% this is the trajectory of the endeffector.

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