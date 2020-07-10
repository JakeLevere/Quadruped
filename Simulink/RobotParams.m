%% Simulink Parameters

%% "Given" Information
o = [0; 4; 0];
R = [1 0 0; 0 1 0; 0 0 1];

rleg= 0.5;

% Constant Robot Parameters
%Constants
leg_min = 4.4434; % prismatic
leg_max = 9.0552; % prismatic

top_diam = 5.23;
shin_length = 5.7477;
thigh_length = 3.4638;
hip_length = 1.5374;
alpha = 53.64 * ((2*pi)/360);
beta = 54.02 * ((2*pi)/360);

%% Parameters of the home position
% Assume the robot is moved into home position to satisfy these conditions
% before performing any parallel motion

bot_diam = 16.05;

% Effective Top Diameter - Theta1i = 0, placing the hips inline with si

effective_top_diam = top_diam + (2 * hip_length);

%% Find li matrix

si = (effective_top_diam/2) * [cos(alpha) -cos(alpha) -cos(beta) cos(beta);
    0 0 0 0;
    -sin(alpha) -sin(alpha) sin(beta) sin(beta)];
ui = (bot_diam/2) * [cos(alpha) -cos(alpha) -cos(beta) cos(beta);
    0 0 0 0;
    -sin(alpha) -sin(alpha) sin(beta) sin(beta)];
li = o + (R*si) - ui;



uitest = ((top_diam+(2*hip_length)+(2*thigh_length))/2) * [cos(alpha) -cos(alpha) -cos(beta) cos(beta);
    0 0 0 0;
    -sin(alpha) -sin(alpha) sin(beta) sin(beta)];

u1 = uitest(:,1);
u2 = uitest(:,2);
u3 = uitest(:,3);
u4 = uitest(:,4);

dist12 = abs(norm((si(:,1)) - (si(:,2))));
dist23 = abs(norm((si(:,2)) - (si(:,3))));
dist34 = abs(norm((si(:,3)) - (si(:,4))));
dist41 = abs(norm((si(:,4)) - (si(:,1))));