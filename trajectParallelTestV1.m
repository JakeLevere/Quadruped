%% trajectory test parallel movements walking robot
clc; clear all; close all;
%%
homePose = [0;0; 5.7477;0;0;0];
[AlphaiHome,BetaiHome,GammaiHome,UHome, hipJntPosHome,kneeJntPosHome,ankleJntPosHome] = InverseKinematicsParallelWalkingV2(homePose, 'XYZ', true);
% saveas(figHome,'Parallel Movement HomePose','png');

% format: columns are the legs, rows are the angles alpha;beta;gamma(3x4)
homeAnglesMat = [AlphaiHome;BetaiHome;GammaiHome]


goalPose = [0;0;8;0;0;0];
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
% constans for the cubic trajectory for the endeffector
[xTrajContA0,xTrajContA1,xTrajContA2,xTrajContA3] = cubicTrajectConstsTaskSpace(t0,tf,homePose(1),goalPose(1),v0,vf);
[yTrajContA0,yTrajContA1,yTrajContA2,yTrajContA3] = cubicTrajectConstsTaskSpace(t0,tf,homePose(2),goalPose(2),v0,vf);
[zTrajContA0,zTrajContA1,zTrajContA2,zTrajContA3] = cubicTrajectConstsTaskSpace(t0,tf,homePose(3),goalPose(3),v0,vf);



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
    
    currPose = [xPosCurr;yPosCurr;zPosCurr;0;0;0]; % currently have orientation as 0,because not sure how to get the orientation at a specific time
    listOfPoses(:,i) = currPose;
    trajectPositions(:,i) = currPose(1:3);
end

%  % each position will then be every 3 rows. so size is (3*length(listofposes)x4)
% listOfHipJntPos = [];
% listOfKneeJntPos = [];
% listOfAnkleJntPos = [];

% joint positions for each leg
% Leg 1
listOfHipJntPosLeg1 = zeros(3,length(listOfPoses));
listOfKneeJntPosLeg1 = zeros(3,length(listOfPoses));
listOfAnkleJntPosLeg1 = zeros(3,length(listOfPoses));
% Leg 2
listOfHipJntPosLeg2 = zeros(3,length(listOfPoses));
listOfKneeJntPosLeg2 = zeros(3,length(listOfPoses));
listOfAnkleJntPosLeg2 = zeros(3,length(listOfPoses));
% Leg 3
listOfHipJntPosLeg3 = zeros(3,length(listOfPoses));
listOfKneeJntPosLeg3 = zeros(3,length(listOfPoses));
listOfAnkleJntPosLeg3 = zeros(3,length(listOfPoses));
% Leg 4
listOfHipJntPosLeg4 = zeros(3,length(listOfPoses));
listOfKneeJntPosLeg4 = zeros(3,length(listOfPoses));
listOfAnkleJntPosLeg4 = zeros(3,length(listOfPoses));


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
    listOfHipJntPosLeg1(:,i) = hipJntPosCurr(:,1);
    listOfKneeJntPosLeg1(:,i)  = kneeJntPosCurr(:,1);
    listOfAnkleJntPosLeg1(:,i)  = ankleJntPosCurr(:,1);
    % Leg 2 joints
    listOfHipJntPosLeg2(:,i) = hipJntPosCurr(:,2);
    listOfKneeJntPosLeg2(:,i)  = kneeJntPosCurr(:,2);
    listOfAnkleJntPosLeg2(:,i)  = ankleJntPosCurr(:,2);
    % Leg 3 joints
    listOfHipJntPosLeg3(:,i) = hipJntPosCurr(:,3);
    listOfKneeJntPosLeg3(:,i)  = kneeJntPosCurr(:,3);
    listOfAnkleJntPosLeg3(:,i)  = ankleJntPosCurr(:,3);
    % Leg 4 joints
    listOfHipJntPosLeg4(:,i) = hipJntPosCurr(:,4);
    listOfKneeJntPosLeg4(:,i)  = kneeJntPosCurr(:,4);
    listOfAnkleJntPosLeg4(:,i)  = ankleJntPosCurr(:,4);
end


fig = figure('Name', 'Animated of Walking Robot');
grid on
xlim([-10,10])
ylim([-10,10])
zlim([0,10])
xlabel('X Position (in)');
zlabel('Z Position (in)');
ylabel('Y Position (in)');
title('Stickplot of Walking Robot')
% filename= '30-35-60-front.gif';
view(30,45);

trajectLine = animatedline('Color','m');
for i=1:length(trajectPositions)
    currPos = trajectPositions(:,i); % plot the trajectory
    addpoints(trajectLine,currPos(1),currPos(2),currPos(3));% plot the trajectory
    drawnow
    pause(0.01);
    
end



% calculate the inverse kinematics
%     [AlphaiCurr,BetaiCurr,GammaiCurr,UCurr, hipJntPosCurr,kneeJntPosCurr,ankleJntPosCurr] = InverseKinematicsParallelWalkingV2(currPose, 'XYZ', true);

% % get cubic trajectory constants
% % format: rows are joints alpha; beta; gamma, columns are a values a0-a3 (3x4)
% leg1Consts =zeros(3,4);
% leg2Consts =zeros(3,4);
% leg3Consts =zeros(3,4);
% leg4Consts =zeros(3,4);
% for i=1:4 % for each leg
%     for k=1:3
%         switch i
%             case 1 % leg 1
%                 leg1Consts(k,:) = cubicTrajectConstsV1(t0,tf,homeAnglesMat(k,i),goalAnglesMat(k,i),v0,vf);
%             case 2 % leg 2
%                 leg2Consts(k,:) = cubicTrajectConstsV1(t0,tf,homeAnglesMat(k,i),goalAnglesMat(k,i),v0,vf);
%             case 3% leg 3
%                 leg3Consts(k,:) = cubicTrajectConstsV1(t0,tf,homeAnglesMat(k,i),goalAnglesMat(k,i),v0,vf);
%             case 4 % leg 4
%                 leg4Consts(k,:) = cubicTrajectConstsV1(t0,tf,homeAnglesMat(k,i),goalAnglesMat(k,i),v0,vf);
%         end
%     end
% end
%
% timeframe = linspace(t0,tf,20);
% allLegJointAnglesAtTime = zeros(3,4); % eahc row is alpha, beta, or gamma, columns are the legs
% % get the joint values for each time
% for i= 1:length(timeframe)
%     t = timeframe(i); %get current time
%     % get the q values for each leg and each joint
%     for j=1:4 % each leg
%         for k = 1:3 % each joint
%             switch j
%                 case 1 % leg 1
%                     allLegJointAnglesAtTime(k,j) = cubicTrajectEqn(leg1Consts(1),leg1Consts(2),leg1Consts(3),leg1Consts(4), t);
%                 case 2 % leg 2
%                     allLegJointAnglesAtTime(k,j) = cubicTrajectEqn(leg2Consts(1),leg2Consts(2),leg2Consts(3),leg2Consts(4), t);
%                 case 3 %leg 3
%                     allLegJointAnglesAtTime(k,j) = cubicTrajectEqn(leg3Consts(1),leg3Consts(2),leg3Consts(3),leg3Consts(4), t);
%                 case 4%leg 4
%                     allLegJointAnglesAtTime(k,j) = cubicTrajectEqn(leg4Consts(1),leg4Consts(2),leg4Consts(3),leg4Consts(4), t);
%             end
%         end
%     end
% % we now have all the joint angles at this specific time
% end



