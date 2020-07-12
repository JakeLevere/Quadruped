%% Simulink Parameters

%% "Given" Information
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

%% Find si and ui matrix
botR = (thigh_length)+hip_length+topR

[s, u] = SandUVectors(topR, botR, theta1RadHome, theta1RadHome, false);
dist12 = abs(norm((s(:,1)) - (s(:,2))));
dist24 = abs(norm((s(:,2)) - (s(:,4))));
dist34 = abs(norm((s(:,3)) - (s(:,4))));
dist31 = abs(norm((s(:,3)) - (s(:,1))));

% U vector
u1 = u(:,1);
u2 = u(:,2);
u3 = u(:,3);
u4 = u(:,4);

%% Create Angle ranges

ndivisions = 100;
leg1_alpha = create_range(0, alphai(1), ndivisions, 10);
leg1_beta = create_range(0, betai(1), ndivisions, 10);
leg1_gamma = create_range(0, gammai(1)-90, ndivisions, 10);
leg2_alpha = create_range(0, alphai(2), ndivisions, 10);
leg2_beta = create_range(0, betai, ndivisions, 10);
leg2_gamma = create_range(0, gammai(2)-90, ndivisions, 10);
leg3_alpha = create_range(0, alphai(3), ndivisions, 10);
leg3_beta = create_range(0, betai(3), ndivisions, 10);
leg3_gamma = create_range(0, gammai(3)-90, ndivisions, 10);
leg4_alpha =create_range(0, alphai(4), ndivisions, 10);
leg4_beta = create_range(0, betai(4), ndivisions, 10);
leg4_gamma = create_range(0, gammai(4)-90, ndivisions, 10);
leg1_rholambda = create_range(0, rhoi(1)+lamdai(1)-90, ndivisions, 10);