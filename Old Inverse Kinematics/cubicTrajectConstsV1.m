%% Cubic Trajector Constants Function
% Provide initial and final times, angles, and velocities.
function [a0,a1,a2,a3] = cubicTrajectConstsV1(t0,tf,q0,qf,v0,vf)

b= [q0;v0;qf;vf];

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