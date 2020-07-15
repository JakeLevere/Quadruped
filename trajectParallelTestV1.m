%% trajectory test parallel movements walking robot
clc; clear all; close all;
%% Initial Positions, angles and values
homePose = [0;0; 5.7477;0;0;0];
[AlphaiHome,BetaiHome,GammaiHome,UHome, hipJntPosHome,kneeJntPosHome,ankleJntPosHome] = InverseKinematicsParallelWalkingV2(homePose, 'XYZ', true);
% saveas(figHome,'Parallel Movement HomePose','png');

% format: columns are the legs, rows are the angles alpha;beta;gamma(3x4)
homeAnglesMat = [AlphaiHome;BetaiHome;GammaiHome]

goalPose =[0;2;5;-5;5;-45];
% goalPose = [0;0;8;0;0;0];
[AlphaiGoal,BetaiGoal,GammaiGoal,UGoal, hipJntPosGoal,kneeJntPosGoal,ankleJntPosGoal] = InverseKinematicsParallelWalkingV2(goalPose, 'XYZ', true);
% saveas(figGoal,'Parallel Movement GoalPose','png');

% format: columns are the legs, rows are the angles alpha;beta;gamma(3x4)
goalAnglesMat = [AlphaiGoal;BetaiGoal;GammaiGoal]

% initial and final times in seconds
t0 = 0;
tf = 5;
% initial and final velocities in inches/second
v0 =0;
vf =0;
% constants for the cubic trajectory for the endeffector
[xTrajContA0,xTrajContA1,xTrajContA2,xTrajContA3] = cubicTrajectConstsTaskSpace(t0,tf,homePose(1),goalPose(1),v0,vf);
[yTrajContA0,yTrajContA1,yTrajContA2,yTrajContA3] = cubicTrajectConstsTaskSpace(t0,tf,homePose(2),goalPose(2),v0,vf);
[zTrajContA0,zTrajContA1,zTrajContA2,zTrajContA3] = cubicTrajectConstsTaskSpace(t0,tf,homePose(3),goalPose(3),v0,vf);

[wxTrajContA0,wxTrajContA1,wxTrajContA2,wxTrajContA3] = cubicTrajectConstsTaskSpace(t0,tf,homePose(4),goalPose(4),v0,vf);
[wyTrajContA0,wyTrajContA1,wyTrajContA2,wyTrajContA3] = cubicTrajectConstsTaskSpace(t0,tf,homePose(5),goalPose(5),v0,vf);
[wzTrajContA0,wzTrajContA1,wzTrajContA2,wzTrajContA3] = cubicTrajectConstsTaskSpace(t0,tf,homePose(6),goalPose(6),v0,vf);



%% get the xyz coordinates and poses for each trajectory
timeframe = linspace(t0,tf,20); % time step array that you move to that position
% get all of the points along the trajectory
listOfPoses = zeros(6,length(timeframe));
trajectPositions = zeros(3,length(timeframe));
for i= 1:length(timeframe)
    t = timeframe(i); %get current time
    % get current x y and z positions a this time
    xPosCurr = cubicTrajectEqn(xTrajContA0,xTrajContA1,xTrajContA2,xTrajContA3, t);
    yPosCurr = cubicTrajectEqn(yTrajContA0,yTrajContA1,yTrajContA2,yTrajContA3, t);
    zPosCurr = cubicTrajectEqn(zTrajContA0,zTrajContA1,zTrajContA2,zTrajContA3, t);
    
    wxCurr = cubicTrajectEqn(wxTrajContA0,wxTrajContA1,wxTrajContA2,wxTrajContA3, t);
    wyCurr = cubicTrajectEqn(wyTrajContA0,wyTrajContA1,wyTrajContA2,wyTrajContA3, t);
    wzCurr = cubicTrajectEqn(wzTrajContA0,wzTrajContA1,wzTrajContA2,wzTrajContA3, t);
    
    currPose = [xPosCurr;yPosCurr;zPosCurr;wxCurr;wyCurr;wzCurr]; % currently have orientation as 0,because not sure how to get the orientation at a specific time
    listOfPoses(:,i) = currPose;
    trajectPositions(:,i) = currPose(1:3);
end

%% get the joint positions and values for each point on the trajecotry
% joint positions for each leg
% rows 1:3 are hip, 4:6 are knee, 7:9 are ankle
listOfJntPosLeg1 = zeros(9,length(listOfPoses));
listOfJntPosLeg2 = zeros(9,length(listOfPoses));
listOfJntPosLeg3 = zeros(9,length(listOfPoses));
listOfJntPosLeg4 = zeros(9,length(listOfPoses));
% each row is a position, each column is a leg
allAlphai = zeros(length(listOfPoses),4);
allBetai = zeros(length(listOfPoses),4);
allGammai = zeros(length(listOfPoses),4);
for i = 1:length(listOfPoses)
    currPose = listOfPoses(:,i);
    [AlphaiCurr,BetaiCurr,GammaiCurr,~, hipJntPosCurr,kneeJntPosCurr,ankleJntPosCurr] = InverseKinematicsParallelWalkingV2(currPose, 'XYZ', true);
    % get a list of all angles needed for the trajectory
    allAlphai(i,:) = AlphaiCurr;
    allBetai(i,:) = BetaiCurr;
    allGammai(i,:) = GammaiCurr;
    
    % get  alist of points the joints will be at in space for plotting
    % Leg 1 joints
    listOfJntPosLeg1(1:3,i) = hipJntPosCurr(:,1);
    listOfJntPosLeg1(4:6,i)  = kneeJntPosCurr(:,1);
    listOfJntPosLeg1(7:9,i)  = ankleJntPosCurr(:,1);
    
    % Leg 2 joints
    listOfJntPosLeg2(1:3,i) = hipJntPosCurr(:,2);
    listOfJntPosLeg2(4:6,i)  = kneeJntPosCurr(:,2);
    listOfJntPosLeg2(7:9,i)  = ankleJntPosCurr(:,2);
    
    % Leg 3 joints
    listOfJntPosLeg3(1:3,i) = hipJntPosCurr(:,3);
    listOfJntPosLeg3(4:6,i)  = kneeJntPosCurr(:,3);
    listOfJntPosLeg3(7:9,i)  = ankleJntPosCurr(:,3);
    
    % Leg 4 joints
    listOfJntPosLeg4(1:3,i) = hipJntPosCurr(:,4);
    listOfJntPosLeg4(4:6,i)  = kneeJntPosCurr(:,4);
    listOfJntPosLeg4(7:9,i)  = ankleJntPosCurr(:,4);
    
end

%% plot animation
plotAnimation(trajectPositions,listOfJntPosLeg1,listOfJntPosLeg2,listOfJntPosLeg3,listOfJntPosLeg4, UHome,'Stickplot Animation','home_pos_to_upper.gif')

