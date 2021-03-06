%% Inverse Kinematics for Spider Quadruped Robot

%% "Given" Information
o = [0; 4; 0];
R = [1 0 0; 0 1 0; 0 0 1];

rleg= 0.5;
%% Constant Robot Parameters
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

l = zeros(1,length(li));
for leg_num = 1:length(li)
    l(leg_num) = norm(li(:,leg_num));
end

disp("Leg lengths are: ");
disp(l);

dist12 = abs(norm((si(:,1)) - (si(:,2))));
dist23 = abs(norm((si(:,2)) - (si(:,3))));
dist34 = abs(norm((si(:,3)) - (si(:,4))));
dist41 = abs(norm((si(:,4)) - (si(:,1))));
%dist12 = 5;
%dist23 = 6.7;
%dist34 = 5;
%dist41 = 6.7;
%% Find Joint Variables Theta1, Theta2, Theta3

q = zeros(3, length(li));


% Find Theta2
for leg_num = 1:length(li)
    gamma = pi - acos((dot(si(:,leg_num), li(:,leg_num)))/(norm(si(:,leg_num))*norm(li(:,leg_num))));
    inside = (shin_length^2 - thigh_length^2 - l(leg_num)^2)/(-2*thigh_length*l(leg_num));
    q(2,leg_num) = acos(inside) - gamma;
end

% Find Theta3
for leg_num = 1:length(li)
    inside = (l(leg_num)^2 - thigh_length^2 - shin_length^2)/(-2*thigh_length*shin_length);
    q(3,leg_num) = acos(inside)- pi/2;
end

disp("Joint Variables are: ");
disp(q);

%% Plot the configuration
%{
hold on;

% Plot the XYZ and xyz origin
plot3(0,0,0,'o');
plot3(o(1), o(2), o(3),'o');

% Label the axis
xlabel('X');
ylabel('Y');
zlabel('Z');
axis equal;

% Plot the bot_diam circle
foot_circle_x = [bot_diam/2*cos(0:(2*pi)/5000: 2*pi).'];
foot_circle_z = [bot_diam/2*sin(0:(2*pi)/5000: 2*pi).'];
foot_circle = [foot_circle_x  zeros(length(foot_circle_x),1) foot_circle_z];
plot3(foot_circle(:,1), foot_circle(:,2), foot_circle(:,3));


% Plot the length vectors
si = R.'*ui + o
l1_vector = [ui(:,1) si(:,1)];
l2_vector = [ui(:,2) si(:,2)];
l3_vector = [ui(:,3) si(:,3)];
l4_vector = [ui(:,4) si(:,4)];
s1_vector = [o, si(:,1)];
s2_vector = [o, si(:,2)];
s3_vector = [o, si(:,3)];
s4_vector = [o, si(:,4)];

plot3(l1_vector(1,:), l1_vector(2,:), l1_vector(3,:))
plot3(l2_vector(1,:), l2_vector(2,:), l2_vector(3,:))
plot3(l3_vector(1,:), l3_vector(2,:), l3_vector(3,:))
plot3(l4_vector(1,:), l4_vector(2,:), l4_vector(3,:))
plot3(s1_vector(1,:), s1_vector(2,:), s1_vector(3,:))
plot3(s2_vector(1,:), s2_vector(2,:), s2_vector(3,:))
plot3(s3_vector(1,:), s3_vector(2,:), s3_vector(3,:))
plot3(s4_vector(1,:), s4_vector(2,:), s4_vector(3,:))
%}
%% Create "Joint" Range File
ndivisions = 100;
leg1_theta1 = create_range(0, q(1,1), ndivisions, 10);
leg1_theta2 = create_range(0, q(2,1), ndivisions, 10);
leg1_theta3 = create_range(0, q(3,1), ndivisions, 10);
leg2_theta1 = create_range(0, q(1,2), ndivisions, 10);
leg2_theta2 = create_range(0, q(2,2), ndivisions, 10);
leg2_theta3 = create_range(0, q(3,2), ndivisions, 10);
leg3_theta1 = create_range(0, q(1,3), ndivisions, 10);
leg3_theta2 = create_range(0, q(2,3), ndivisions, 10);
leg3_theta3 = create_range(0, q(3,3), ndivisions, 10);
leg4_theta1 = create_range(0, q(1,4), ndivisions, 10);
leg4_theta2 = create_range(0, q(2,4), ndivisions, 10);
leg4_theta3 = create_range(0, q(3,4), ndivisions, 10);


%% Helper Function

function timetbl = create_range(startvalue, endvalue, ndivisions, rate)
range = zeros(1, ndivisions);
index = 1;

if startvalue > endvalue
    index = ndivisions;
    for i = endvalue:(startvalue-endvalue)/(ndivisions-1):startvalue
       range(index) = i;
       index = index-1;
    end
elseif startvalue < endvalue
    for i = startvalue:(endvalue-startvalue)/(ndivisions-1):endvalue
       range(index) = i;
       index = index+1;
    end
    
else %startvalue = endvalue
    for i = 1:ndivisions-1
       range(i) = startvalue; 
    end
end

timetbl = timetable(range.', 'SampleRate', 10)
end