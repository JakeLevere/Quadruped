%% Cubic trajectory equation - by Ethan Lauer
function [x,dx] = cubicTrajectEqn(a0,a1,a2,a3, t)
x = a0+a1*t+a2*t^2+a3*t^3;
dx = a1+2*a2*t+3*a3*(t^2);
end