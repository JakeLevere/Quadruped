%% trajectory test parallel movements walking robot
clc; clear all; close all;
%% Initial Positions, angles and values
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
% constants for the cubic trajectory for the endeffector
[xTrajContA0,xTrajContA1,xTrajContA2,xTrajContA3] = cubicTrajectConstsTaskSpace(t0,tf,homePose(1),goalPose(1),v0,vf);
[yTrajContA0,yTrajContA1,yTrajContA2,yTrajContA3] = cubicTrajectConstsTaskSpace(t0,tf,homePose(2),goalPose(2),v0,vf);
[zTrajContA0,zTrajContA1,zTrajContA2,zTrajContA3] = cubicTrajectConstsTaskSpace(t0,tf,homePose(3),goalPose(3),v0,vf);


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
    
    currPose = [xPosCurr;yPosCurr;zPosCurr;0;0;0]; % currently have orientation as 0,because not sure how to get the orientation at a specific time
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
% fig = figure('Name', 'Animated of Walking Robot');
% grid on
% xlim([-10,10])
% ylim([-10,10])
% zlim([0,10])
% hold on
% xlabel('X Position (in)');
% zlabel('Z Position (in)');
% ylabel('Y Position (in)');
% title('Animated Stickplot of Walking Robot')
% filename= 'home_to_upper.gif';
% view(30,45);
% 
% trajectLine = animatedline('Color','m');
% for i=1:length(trajectPositions)
%     %% plot the trajectory
%     currPos = trajectPositions(:,i); % plot the trajectory
%     addpoints(trajectLine,currPos(1),currPos(2),currPos(3));
%     
%     %% Plot joint and feet points in space
%     % Front legs are light blue, back are dark blue
%     
%     % Leg 1
%     currHipJntPointLeg1=scatter3(listOfJntPosLeg1(1,i),listOfJntPosLeg1(2,i),listOfJntPosLeg1(3,i),'MarkerFaceColor','c');
%     currKneeJntPointLeg1=scatter3(listOfJntPosLeg1(4,i),listOfJntPosLeg1(5,i),listOfJntPosLeg1(6,i),'MarkerFaceColor','c');
%     currAnkleJntPointLeg1=scatter3(listOfJntPosLeg1(7,i),listOfJntPosLeg1(8,i),listOfJntPosLeg1(9,i),'MarkerFaceColor','c');
%     % Leg 2
%     currHipJntPointLeg2=scatter3(listOfJntPosLeg2(1,i),listOfJntPosLeg2(2,i),listOfJntPosLeg2(3,i),'MarkerFaceColor','c');
%     currKneeJntPointLeg2=scatter3(listOfJntPosLeg2(4,i),listOfJntPosLeg2(5,i),listOfJntPosLeg2(6,i),'MarkerFaceColor','c');
%     currAnkleJntPointLeg2=scatter3(listOfJntPosLeg2(7,i),listOfJntPosLeg2(8,i),listOfJntPosLeg2(9,i),'MarkerFaceColor','c');
%     
%     % Leg 3
%     currHipJntPointLeg3=scatter3(listOfJntPosLeg3(1,i),listOfJntPosLeg3(2,i),listOfJntPosLeg3(3,i),'MarkerFaceColor','b');
%     currKneeJntPointLeg3=scatter3(listOfJntPosLeg3(4,i),listOfJntPosLeg3(5,i),listOfJntPosLeg3(6,i),'MarkerFaceColor','b');
%     currAnkleJntPointLeg3=scatter3(listOfJntPosLeg3(7,i),listOfJntPosLeg3(8,i),listOfJntPosLeg3(9,i),'MarkerFaceColor','b');
%     
%     % Leg 4
%     currHipJntPointLeg4=scatter3(listOfJntPosLeg4(1,i),listOfJntPosLeg4(2,i),listOfJntPosLeg4(3,i),'MarkerFaceColor','b');
%     currKneeJntPointLeg4=scatter3(listOfJntPosLeg4(4,i),listOfJntPosLeg4(5,i),listOfJntPosLeg4(6,i),'MarkerFaceColor','b');
%     currAnkleJntPointLeg4=scatter3(listOfJntPosLeg4(7,i),listOfJntPosLeg4(8,i),listOfJntPosLeg4(9,i),'MarkerFaceColor','b');
%     
%     % Feet - putting this in here, might be useful once we start moving the
%     % legs and feet as well
%     feetPosLeg1 = scatter3(UHome(1,1),UHome(2,1), UHome(3,1),'MarkerFaceColor','k');
%     feetPosLeg2 = scatter3(UHome(1,2),UHome(2,2), UHome(3,2),'MarkerFaceColor','k');
%     feetPosLeg3 = scatter3(UHome(1,3),UHome(2,3), UHome(3,3),'MarkerFaceColor','k');
%     feetPosLeg4 = scatter3(UHome(1,4),UHome(2,4), UHome(3,4),'MarkerFaceColor','k');
% 
%     % put points in array so easier to delete
%     jntPointArr = [currHipJntPointLeg1,currKneeJntPointLeg1,currAnkleJntPointLeg1,...
%         currHipJntPointLeg2,currKneeJntPointLeg2,currAnkleJntPointLeg2,...
%         currHipJntPointLeg3,currKneeJntPointLeg3,currAnkleJntPointLeg3,...
%         currHipJntPointLeg4,currKneeJntPointLeg4,currAnkleJntPointLeg4,...
%         feetPosLeg1,feetPosLeg2, feetPosLeg3, feetPosLeg4];
%     %% Plot the lines connecting the joints
%     % connect hips
%     hip1To2Line = plot3([listOfJntPosLeg1(1,i);listOfJntPosLeg2(1,i)],...
%         [listOfJntPosLeg1(2,i);listOfJntPosLeg2(2,i)],...
%         [listOfJntPosLeg1(3,i);listOfJntPosLeg2(3,i)],'r-');
%     hip2To4Line = plot3([listOfJntPosLeg2(1,i);listOfJntPosLeg4(1,i)],...
%         [listOfJntPosLeg2(2,i);listOfJntPosLeg4(2,i)],...
%         [listOfJntPosLeg2(3,i);listOfJntPosLeg4(3,i)],'r-');
%     hip3To4Line = plot3([listOfJntPosLeg3(1,i);listOfJntPosLeg4(1,i)],...
%         [listOfJntPosLeg3(2,i);listOfJntPosLeg4(2,i)],...
%         [listOfJntPosLeg3(3,i);listOfJntPosLeg4(3,i)],'r-');
%     hip3To1Line = plot3([listOfJntPosLeg3(1,i);listOfJntPosLeg1(1,i)],...
%         [listOfJntPosLeg3(2,i);listOfJntPosLeg1(2,i)],...
%         [listOfJntPosLeg3(3,i);listOfJntPosLeg1(3,i)],'r-');
%     
%     % connect hips to knees
%     hipToKneeLineLeg1 = plot3([listOfJntPosLeg1(1,i);listOfJntPosLeg1(4,i)],...
%         [listOfJntPosLeg1(2,i);listOfJntPosLeg1(5,i)],...
%         [listOfJntPosLeg1(3,i);listOfJntPosLeg1(6,i)],'r-');
%     hipToKneeLineLeg2 = plot3([listOfJntPosLeg2(1,i);listOfJntPosLeg2(4,i)],...
%         [listOfJntPosLeg2(2,i);listOfJntPosLeg2(5,i)],...
%         [listOfJntPosLeg2(3,i);listOfJntPosLeg2(6,i)],'r-');
%     hipToKneeLineLeg3 = plot3([listOfJntPosLeg3(1,i);listOfJntPosLeg3(4,i)],...
%         [listOfJntPosLeg3(2,i);listOfJntPosLeg3(5,i)],...
%         [listOfJntPosLeg3(3,i);listOfJntPosLeg3(6,i)],'r-');
%     hipToKneeLineLeg4 = plot3([listOfJntPosLeg4(1,i);listOfJntPosLeg4(4,i)],...
%         [listOfJntPosLeg4(2,i);listOfJntPosLeg4(5,i)],...
%         [listOfJntPosLeg4(3,i);listOfJntPosLeg4(6,i)],'r-');
%     
%     
%     % connect knees to ankles
%     kneeToAnkleLineLeg1 = plot3([listOfJntPosLeg1(7,i);listOfJntPosLeg1(4,i)],...
%         [listOfJntPosLeg1(8,i);listOfJntPosLeg1(5,i)],...
%         [listOfJntPosLeg1(9,i);listOfJntPosLeg1(6,i)],'r-');
%     kneeToAnkleLineLeg2 = plot3([listOfJntPosLeg2(7,i);listOfJntPosLeg2(4,i)],...
%         [listOfJntPosLeg2(8,i);listOfJntPosLeg2(5,i)],...
%         [listOfJntPosLeg2(9,i);listOfJntPosLeg2(6,i)],'r-');
%     kneeToAnkleLineLeg3 = plot3([listOfJntPosLeg3(7,i);listOfJntPosLeg3(4,i)],...
%         [listOfJntPosLeg3(8,i);listOfJntPosLeg3(5,i)],...
%         [listOfJntPosLeg3(9,i);listOfJntPosLeg3(6,i)],'r-');
%     kneeToAnkleLineLeg4 = plot3([listOfJntPosLeg4(7,i);listOfJntPosLeg4(4,i)],...
%         [listOfJntPosLeg4(8,i);listOfJntPosLeg4(5,i)],...
%         [listOfJntPosLeg4(9,i);listOfJntPosLeg4(6,i)],'r-');
%     
%     % connect ankles to feet
%      ankleToFeetLineLeg1 = plot3([listOfJntPosLeg1(7,i);UHome(1,1)],...
%         [listOfJntPosLeg1(8,i);UHome(2,1)],...
%         [listOfJntPosLeg1(9,i);UHome(3,1)],'r-');
%     ankleToFeetLineLeg2 = plot3([listOfJntPosLeg2(7,i);UHome(1,2)],...
%         [listOfJntPosLeg2(8,i);UHome(2,2)],...
%         [listOfJntPosLeg2(9,i);UHome(3,2)],'r-');
%     ankleToFeetLineLeg3 = plot3([listOfJntPosLeg3(7,i);UHome(1,3)],...
%         [listOfJntPosLeg3(8,i);UHome(2,3)],...
%         [listOfJntPosLeg3(9,i);UHome(3,3)],'r-');
%     ankleToFeetLineLeg4 = plot3([listOfJntPosLeg4(7,i);UHome(1,4)],...
%         [listOfJntPosLeg4(8,i);UHome(2,4)],...
%         [listOfJntPosLeg4(9,i);UHome(3,4)],'r-');
%     
%     % put in array so easier to delete afterwards
%     lineArr = [hip1To2Line,hip2To4Line,hip3To4Line hip3To1Line,...
%         hipToKneeLineLeg1, hipToKneeLineLeg2, hipToKneeLineLeg3,...
%         hipToKneeLineLeg4,kneeToAnkleLineLeg1, kneeToAnkleLineLeg2,...
%         kneeToAnkleLineLeg3,kneeToAnkleLineLeg4, ankleToFeetLineLeg1,...
%         ankleToFeetLineLeg2, ankleToFeetLineLeg3, ankleToFeetLineLeg4];
%     %% draw, save for giv and pause
%     drawnow
%     frame=getframe(fig);
%     im=frame2im(frame);
%     [imind,cm]=rgb2ind(im,256);
%     if i==1
%         imwrite(imind,cm,filename,'gif','Loopcount',inf);
%     else
%         imwrite(imind,cm,filename,'gif','WriteMode','append');
%     end
%     
%     pause(0.01);
%     %% Delete the previous points and lines unless its the last time point
%     if i ~= length(trajectPositions)
%         for k=1:length(jntPointArr)
%             delete(jntPointArr(k))
%         end
%         for k=1:length(lineArr)
%             delete(lineArr(k))
%         end
%     end
% end

