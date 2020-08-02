%% Parallel Movement of a Walking Robot With Trajectory - by Ethan Lauer
% Given: Home pose(6x1), goal pose (6x1),
% boolean true if degrees false if radians, initial and final times t0, and
% tf, the time step for each of those position (tf) and initial and final
% velocities vo & vf, Title name (ex. 'stickplot'),
% and file name to save gif of animation (ex. 'pose_to_pose.gif')
% Result: allAlphai,allBetai,allGammai - all values of alpha, beta, and
% gamma that each leg needs ot move to to follow the trajectory.
%
% Units: inches, inches/sec, seconds
% NOTE: NO PLOT THIS TIME
function [allAlphai,allBetai,allGammai,supJntPosLeg1,supJntPosLeg2,supJntPosLeg3,supJntPosLeg4] = parallelMoveTrajectV2(homePose, goalPose,t0,tf,tstep,v0,vf)
%% Initial Positions, angles and values
[AlphaiHome,BetaiHome,GammaiHome,UHome, ~,~,~,~] = InverseKinematicsParallelWalkingV2(homePose);

% format: columns are the legs, rows are the angles alpha;beta;gamma(3x4)
% homeAnglesMat = [AlphaiHome;BetaiHome;GammaiHome];

[AlphaiGoal,BetaiGoal,GammaiGoal,~,~,~,~,~] = InverseKinematicsParallelWalkingV2(goalPose);

% format: columns are the legs, rows are the angles alpha;beta;gamma(3x4)
goalAnglesMat = [AlphaiGoal;BetaiGoal;GammaiGoal];

% time step array that you move to that position
timeframe = linspace(t0,tf,tstep);

% constants for the cubic trajectory for the endeffector
% Positions
[xA0,xA1,xA2,xA3] = cubicTrajectConstsTaskSpace(t0,tf,homePose(1),goalPose(1),v0,vf);
[yA0,yA1,yA2,yA3] = cubicTrajectConstsTaskSpace(t0,tf,homePose(2),goalPose(2),v0,vf);
[zA0,zA1,zA2,zA3] = cubicTrajectConstsTaskSpace(t0,tf,homePose(3),goalPose(3),v0,vf);
% orientations
[wxA0,wxA1,wxA2,wxA3] = cubicTrajectConstsTaskSpace(t0,tf,homePose(4),goalPose(4),v0,vf);
[wyA0,wyA1,wyA2,wyA3] = cubicTrajectConstsTaskSpace(t0,tf,homePose(5),goalPose(5),v0,vf);
[wzA0,wzA1,wzA2,wzA3] = cubicTrajectConstsTaskSpace(t0,tf,homePose(6),goalPose(6),v0,vf);

%% get the xyz coordinates and poses for each trajectory
% get all of the Poses along the trajectory over the timeframe
listOfPoses = zeros(6,length(timeframe));
for i= 1:length(timeframe)
    t = timeframe(i); %get current time
    % get current x y and z positions a this time
    xPosCurr = cubicTrajectEqn(xA0,xA1,xA2,xA3, t);
    yPosCurr = cubicTrajectEqn(yA0,yA1,yA2,yA3, t);
    zPosCurr = cubicTrajectEqn(zA0,zA1,zA2,zA3, t);
    % get current wx wy and wz orientations a this time
    wxCurr = cubicTrajectEqn(wxA0,wxA1,wxA2,wxA3, t);
    wyCurr = cubicTrajectEqn(wyA0,wyA1,wyA2,wyA3, t);
    wzCurr = cubicTrajectEqn(wzA0,wzA1,wzA2,wzA3, t);
    
    % Store current pose in list
    listOfPoses(:,i) = [xPosCurr;yPosCurr;zPosCurr;wxCurr;wyCurr;wzCurr];
end

%% get the joint positions and values for each point on the trajecotry

% joint positions for each leg
% rows 1:3 are hip, 4:6 are knee, 7:9 are ankle
supJntPosLeg1 = zeros(9,tstep);
supJntPosLeg2 = zeros(9,tstep);
supJntPosLeg3 = zeros(9,tstep);
supJntPosLeg4 = zeros(9,tstep);

% each row is a position, each column is a leg
allAlphai = zeros(tstep,4);
allBetai = zeros(tstep,4);
allGammai = zeros(tstep,4);
for i = 1:tstep
    currPose = listOfPoses(:,i);
    [AlphaiCurr,BetaiCurr,GammaiCurr,~, hipJntPosCurr,kneeJntPosCurr,ankleJntPosCurr,footJntPosCurr] = InverseKinematicsParallelWalkingV2(currPose);
    % get a list of all angles needed for the trajectory
    allAlphai(i,:) = AlphaiCurr;
    allBetai(i,:) = BetaiCurr;
    allGammai(i,:) = GammaiCurr;
    
    % get  alist of points the joints will be at in space for plotting
    % Leg 1 joints
    supJntPosLeg1(1:3,i) = hipJntPosCurr(:,1);
    supJntPosLeg1(4:6,i)  = kneeJntPosCurr(:,1);
    supJntPosLeg1(7:9,i)  = ankleJntPosCurr(:,1);
    supJntPosLeg1(10:12,i)  = footJntPosCurr(:,1);

    % Leg 2 joints
    supJntPosLeg2(1:3,i) = hipJntPosCurr(:,2);
    supJntPosLeg2(4:6,i)  = kneeJntPosCurr(:,2);
    supJntPosLeg2(7:9,i)  = ankleJntPosCurr(:,2);
    supJntPosLeg2(10:12,i)  = footJntPosCurr(:,2);

    % Leg 3 joints
    supJntPosLeg3(1:3,i) = hipJntPosCurr(:,3);
    supJntPosLeg3(4:6,i)  = kneeJntPosCurr(:,3);
    supJntPosLeg3(7:9,i)  = ankleJntPosCurr(:,3);
    supJntPosLeg3(10:12,i)  = footJntPosCurr(:,3);

    % Leg 4 joints
    supJntPosLeg4(1:3,i) = hipJntPosCurr(:,4);
    supJntPosLeg4(4:6,i)  = kneeJntPosCurr(:,4);
    supJntPosLeg4(7:9,i)  = ankleJntPosCurr(:,4);
    supJntPosLeg4(10:12,i)  = footJntPosCurr(:,4);

end
% these are the list of values the robot will be going through
if isequal(allAlphai(length(allAlphai),:),goalAnglesMat(1,:)) || ...
        isequal(allBetai(length(allBetai),:),goalAnglesMat(2,:)) || ...
        isequal(allGammai(length(allGammai),:),goalAnglesMat(3,:))
    disp('The final angles were reached')
else
    error('The final angles were NOT reached')
end

allAlphai = allAlphai'; % return so each row is a leg and columns are time steps
allBetai = allBetai';
allGammai = allGammai';
end