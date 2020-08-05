%% Parallel Movement of a Walking Robot With Trajectory - by Ethan Lauer
% This function produces the list of alpha, beta, and gamma values for each
% leg of the robot for all of the positions along the trajectory. This also
% calculates the joint positions so the robot can be plotted moving in the
% trajectory.
%
% Units: inches, inches/sec, seconds, (caluclations in radians)
%
% Input: homePose - home position and orientation the robot is starting at
%                   for the parallel movement (6x1 where the first 3 rows 
%                   are the xyz position and the last three rows are the 
%                   rotation wx wx wz
%        goalPose - goal position and orientation the robot is moving to 
%                   for the parallel movement (6x1 where the first 3 rows 
%                   are the xyz position and the last three rows are the 
%                   rotation wx wx wz
%        isDeg - boolean true if providing orientation in degrees
%        t0 - starting time for the trajectory
%        tf - final time where the trajecotry ends
%        tstep - number of points along the trajectory to generate (time
%               step)
%        v0 - initial velocity of the endeffector
%        vf - final velocity of the endeffectory
%        Title - string for the title of the figure of the stickplot
%                (ex. 'stickplot')
%        filename - string of the file name to save the gif as (must end in
%                   '.gif' ex. 'pose_to_pose.gif')
%
% Output:  allAlphai - tstep x 4 matrix of alpha values for each leg (rad)
%                      each column corresponds with the specified leg number
%          allBetai - tstep x 4 matrix of beta values for each leg (rad)
%                      each column corresponds with the specified leg number
%          allGammai - tstep x 4 matrix of gamma values for each leg (rad)
%                      each column corresponds with the specified leg number

function [allAlphai,allBetai,allGammai] = parallelMoveTrajectV1(homePose, goalPose,isDeg,t0,tf,tstep,v0,vf,Title,filename)
%% 
% Convert the angles to radians if provided in degrees so all calculations
% are completed in radians
if isDeg
    homePose = [homePose(1:3); deg2rad(homePose(4:6))];
    goalPose = [goalPose(1:3); deg2rad(goalPose(4:6))];
end

%% 
% General Constants (initial and final values and trajectory constants)

% Initial Positions, angles and values
[AlphaiHome,BetaiHome,GammaiHome,UHome, ~,~,~] = InverseKinematicsParallelWalkingV2(homePose);
% format: columns are the legs, rows are the angles alpha;beta;gamma(3x4)
homeAnglesMat = [AlphaiHome;BetaiHome;GammaiHome];

% Final Positions, angles and values
[AlphaiGoal,BetaiGoal,GammaiGoal,~,~,~,~] = InverseKinematicsParallelWalkingV2(goalPose);
% format: columns are the legs, rows are the angles alpha;beta;gamma(3x4)
goalAnglesMat = [AlphaiGoal;BetaiGoal;GammaiGoal];

% time step array that you move to that position
timeframe = linspace(t0,tf,tstep);

% Constants for the cubic trajectory for the endeffector
% Positions
[xA0,xA1,xA2,xA3] = cubicTrajectConsts(t0,tf,homePose(1),goalPose(1),v0,vf);
[yA0,yA1,yA2,yA3] = cubicTrajectConsts(t0,tf,homePose(2),goalPose(2),v0,vf);
[zA0,zA1,zA2,zA3] = cubicTrajectConsts(t0,tf,homePose(3),goalPose(3),v0,vf);
% orientations
[wxA0,wxA1,wxA2,wxA3] = cubicTrajectConsts(t0,tf,homePose(4),goalPose(4),v0,vf);
[wyA0,wyA1,wyA2,wyA3] = cubicTrajectConsts(t0,tf,homePose(5),goalPose(5),v0,vf);
[wzA0,wzA1,wzA2,wzA3] = cubicTrajectConsts(t0,tf,homePose(6),goalPose(6),v0,vf);

%%
% Get all of the ende effecotr Poses along the trajectory
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

%%
% Get the joint positions and values for each point on the trajecotry

% joint positions for each leg
% rows 1:3 are hip, 4:6 are knee, 7:9 are ankle (foot positions remain for
% parallel mechanisms)
listOfJntPosLeg1 = zeros(9,length(listOfPoses));
listOfJntPosLeg2 = zeros(9,length(listOfPoses));
listOfJntPosLeg3 = zeros(9,length(listOfPoses));
listOfJntPosLeg4 = zeros(9,length(listOfPoses));

% each row is a position, each column  a leg
allAlphai = zeros(length(listOfPoses),4);
allBetai = zeros(length(listOfPoses),4);
allGammai = zeros(length(listOfPoses),4);
for i = 1:length(listOfPoses)
    currPose = listOfPoses(:,i);
    [AlphaiCurr,BetaiCurr,GammaiCurr,~, hipJntPosCurr,kneeJntPosCurr,ankleJntPosCurr] = InverseKinematicsParallelWalkingV2(currPose);
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
% Check to be sure the final angles were reached
if isequal(allAlphai(length(allAlphai),:),goalAnglesMat(1,:)) || ...
        isequal(allBetai(length(allBetai),:),goalAnglesMat(2,:)) || ...
        isequal(allGammai(length(allGammai),:),goalAnglesMat(3,:))
    disp('The final angles were reached')
else
    error('The final angles were NOT reached')
    
end

%%
% Plot the robot moving in this trajectory
plotAnimation(listOfPoses(1:3,:),listOfJntPosLeg1,listOfJntPosLeg2,listOfJntPosLeg3,listOfJntPosLeg4, UHome,Title,filename)
end