%% Leg Sine Trajectory Planning  - Ethan Lauer
% This function generates the list of positions and velocities of the foot
% when moving in a sine wave wrt gnd.
%
% Input: L -stride length (in)
%       maxFootH - max foot height (in)
%       timeMat - vector of time steps where trajectory is evaluated
%
% Output: posF_g - positions of the foot wrt gnd over the full time (2xn
%                   where row 1 is x or y (depnding on direction of walking) and row 2 is z
%           velF_g - velocities of the foot wrt gnd over the full time (2xn
%                   where row 1 is x or y (depnding on direction of walking) and row 2 is z

function [posF_g,velF_g] = legTrajSine(L,maxFootH,timeMat)
t0= timeMat(1);
t4 = timeMat(end);

v0 =0;
vf =0;
y0 =-L/2;
yf = L/2;

[ya0,ya1,ya2,ya3] = cubicTrajectConsts(t0,t4,y0,yf,v0,vf);

w = pi/L; % frequency so only looking at positive sine wave
% timeInterval = linspace(t0,t4);
for i = 1:length(timeMat)
    t = timeMat(i);
    % get the y pos and vel values at this time period
    [yval, dyval] = cubicTrajectEqn(ya0,ya1,ya2,ya3, t);
    % row 1 is y position, row 2 is z position
    posF_g(:,i) = [yval;maxFootH*sin((w*yval)+(pi/2))];
    velF_g(:,i) = [dyval;maxFootH*w*dyval*cos(w*yval)];
end
end