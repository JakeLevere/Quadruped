%% Final Project Inverse Kinematics test Parallel Movements
clc; clear all; close all;

%%
% See Attached CAD Pictures for Details
% The following dimensions are in inches:

numLegs = 4;
% % Leg Lengths
leg_min = 4.4434; % for prismatic 6.2
leg_max = 9.0552;% for prismatic 10.3

% Leg Link Lengths
coxaL1 =1.5374;
femurL2 = 3.4638;
tibiaL3 = 5.7477;

% Platform diamters/Radii
top_diam = 5.23;
bot_diam = 16.05;

topR = top_diam/2;
botR = bot_diam/2;

% Home angles
theta1DegHome = 53.64;% degrees
theta1RadHome = deg2rad(theta1DegHome);% radians
theta2DegHome = 54.02;% degrees
theta2RadHome = deg2rad(theta2DegHome);% radians

%% Setting up S and U Vectors
% Each vector is wrt its own frame (S in {c} and U in {o})
[Si1, U] = SandUVectors(topR, botR, theta1DegHome, theta2DegHome, true)

%% O Vector and Rotation matrix
% for motion straight up and down with no rotation theta1 or theta 3
% in fully crouched position low as it can go (2.18) (beware of imaginary
% numbers
% in fully extended position high as it can go(8.19)


%% Inverse Kinematics
% [LvectPrime,LmagPrime] = invKin(poseDownMin,si2min,U,'XYZ',true)
poseDes = [0;0;3;0;0;0];
[alphai,betai,lamdai,gammai,phii,rhoi,Li_vect,Li_prime_vect,si2] = invKinWalking(poseDes,Si1,U,coxaL1,femurL2,tibiaL3,'XYZ',true)



%% Plotting

% joint position for plotting
hipJntPos = U+Li_vect;
kneeJntPos = U+Li_prime_vect;
for i=1:numLegs
    ankleAngle = betai(i)+phii(i);
    hyp = femurL2*cosd(ankleAngle); % make into radians if needed (create function for plotting these positions)
    ankleZ = femurL2*sind(ankleAngle);
    switch i
        case 1
            xyangle = theta1DegHome-alphai(i);
            ankleX = -hyp*cosd(xyangle);
            ankleY = hyp*sind(xyangle);
            
        case 2
            xyangle = theta1DegHome+alphai(i);
            ankleX = hyp*cosd(xyangle);
            ankleY = hyp*sind(xyangle);
        case 3
            xyangle = theta2DegHome+alphai(i);
            ankleX = -hyp*cosd(xyangle);
            ankleY = -hyp*sind(xyangle);
        case 4
            xyangle = theta2DegHome-alphai(i);
            ankleX = hyp*cosd(xyangle);
            ankleY = -hyp*sind(xyangle);
    end
    ankleJntPos(:,i) = kneeJntPos(:,i) +[ankleX;ankleY;ankleZ];
end
% figure
figure('Name', 'Stickplot of Walking Robot')
t=-pi:0.01:pi;
botCircleX=botR*cos(t);
botCircleY=botR*sin(t);
plot3(botCircleX,botCircleY,zeros(1,numel(botCircleX)),'r--')% circle representing bot platform
hold on
plot3(0,0,0,'*k') % origin
hold on
% Hip joint positions
for i =1:numLegs
    plot3(U(1,i),U(2,i),U(3,i),'*b');
    hold on
    plot3(hipJntPos(1,i),hipJntPos(2,i),hipJntPos(3,i),'*b-');
    hold on
    plot3(kneeJntPos(1,i),kneeJntPos(2,i),kneeJntPos(3,i),'*g-');
    hold on
    plot3(ankleJntPos(1,i),ankleJntPos(2,i),ankleJntPos(3,i),'*m-');
    hold on
    
end

for i=1:numLegs-2
    plot3([hipJntPos(1,i);hipJntPos(1,i+2)],...
        [hipJntPos(2,i);hipJntPos(2,i+2)],...
        [hipJntPos(3,i);hipJntPos(3,i+2)],'r-');
    hold on
end
plot3([hipJntPos(1,4);hipJntPos(1,3)],...
    [hipJntPos(2,4);hipJntPos(2,3)],...
    [hipJntPos(3,4);hipJntPos(3,3)],'r-');
hold on
plot3([hipJntPos(1,2);hipJntPos(1,1)],...
    [hipJntPos(2,2);hipJntPos(2,1)],...
    [hipJntPos(3,2);hipJntPos(3,1)],'r-');
hold on



% % lines of the Li vectors
for i=1:numLegs
%     plot3([U(1,i);hipJntPos(1,i)],...
%         [U(2,i);hipJntPos(2,i)],...
%         [U(3,i);hipJntPos(3,i)],'k--');
%     hold on
%     % lines of the Li prime vectors
%     plot3([U(1,i);kneeJntPos(1,i)],...
%         [U(2,i);kneeJntPos(2,i)],...
%         [U(3,i);kneeJntPos(3,i)],'k--');
%     hold on
    
    % lines connecting the hip and knee
    plot3([hipJntPos(1,i);kneeJntPos(1,i)],...
        [hipJntPos(2,i);kneeJntPos(2,i)],...
        [hipJntPos(3,i);kneeJntPos(3,i)],'r-');
    hold on
    
    % lines connecting the ankle and knee
    plot3([ankleJntPos(1,i);kneeJntPos(1,i)],...
        [ankleJntPos(2,i);kneeJntPos(2,i)],...
        [ankleJntPos(3,i);kneeJntPos(3,i)],'r-');
    hold on
    
    
    % lines connecting the ankle and foot
    plot3([U(1,i);ankleJntPos(1,i)],...
        [U(2,i);ankleJntPos(2,i)],...
        [U(3,i);ankleJntPos(3,i)],'r-');
    hold on
    
    
end



grid on
xlim([-10,10])
ylim([-10,10])
zlim([0,10])
xlabel('X Position (in)');
zlabel('Z Position (in)');
ylabel('Y Position (in)');
title('StickPlot')
