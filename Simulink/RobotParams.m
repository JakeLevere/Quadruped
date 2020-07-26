%% Simulink Parameters

%% "Given" Information
o = [0; 0; 8; 0; 0; 0];

% Simulink Model Parameters
rleg= 0.5;

% Constant Robot Parameters
shin_length = 5.7477;
thigh_length = 3.4638;
hip_length = 1.5374;

top_diam = 5.23;
bot_diam = top_diam+(2*(hip_length+thigh_length));
topR = top_diam/2;
botR = bot_diam/2;

theta1DegHome = 53.64;% degrees
theta2DegHome = 54.02;% degrees
alpha = 53.64 * ((2*pi)/360);
beta = 54.02 * ((2*pi)/360);

%% Find si and ui matrix
theta1RadHome = deg2rad(53.64);% radians
theta2RadHome = deg2rad(54.02);% radians
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
[alphai, betai, gammai, ~, ~, ~, ~] = InverseKinematicsParallelWalkingV2(o, true);

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
