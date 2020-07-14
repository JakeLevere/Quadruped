%% plot animation
% this function plots the animation of the robot moving as a parallel
% mechanism and saves it as a gif.
function [] =plotAnimation(trajectPositions,listOfJntPosLeg1,listOfJntPosLeg2,listOfJntPosLeg3,listOfJntPosLeg4, UHome, Title, fileName)
fig = figure('Name', Title);
grid on
xlim([-10,10])
ylim([-10,10])
zlim([0,10])
hold on
xlabel('X Position (in)');
zlabel('Z Position (in)');
ylabel('Y Position (in)');
title(Title)
filename= fileName;
view(30,45);

trajectLine = animatedline('Color','m');
for i=1:length(trajectPositions)
    %% plot the trajectory
    currPos = trajectPositions(:,i); % plot the trajectory
    addpoints(trajectLine,currPos(1),currPos(2),currPos(3));
    
    %% Plot joint and feet points in space
    % Front legs are light blue, back are dark blue
    
    % Leg 1
    currHipJntPointLeg1=scatter3(listOfJntPosLeg1(1,i),listOfJntPosLeg1(2,i),listOfJntPosLeg1(3,i),'MarkerFaceColor','c');
    currKneeJntPointLeg1=scatter3(listOfJntPosLeg1(4,i),listOfJntPosLeg1(5,i),listOfJntPosLeg1(6,i),'MarkerFaceColor','c');
    currAnkleJntPointLeg1=scatter3(listOfJntPosLeg1(7,i),listOfJntPosLeg1(8,i),listOfJntPosLeg1(9,i),'MarkerFaceColor','c');
    % Leg 2
    currHipJntPointLeg2=scatter3(listOfJntPosLeg2(1,i),listOfJntPosLeg2(2,i),listOfJntPosLeg2(3,i),'MarkerFaceColor','c');
    currKneeJntPointLeg2=scatter3(listOfJntPosLeg2(4,i),listOfJntPosLeg2(5,i),listOfJntPosLeg2(6,i),'MarkerFaceColor','c');
    currAnkleJntPointLeg2=scatter3(listOfJntPosLeg2(7,i),listOfJntPosLeg2(8,i),listOfJntPosLeg2(9,i),'MarkerFaceColor','c');
    
    % Leg 3
    currHipJntPointLeg3=scatter3(listOfJntPosLeg3(1,i),listOfJntPosLeg3(2,i),listOfJntPosLeg3(3,i),'MarkerFaceColor','b');
    currKneeJntPointLeg3=scatter3(listOfJntPosLeg3(4,i),listOfJntPosLeg3(5,i),listOfJntPosLeg3(6,i),'MarkerFaceColor','b');
    currAnkleJntPointLeg3=scatter3(listOfJntPosLeg3(7,i),listOfJntPosLeg3(8,i),listOfJntPosLeg3(9,i),'MarkerFaceColor','b');
    
    % Leg 4
    currHipJntPointLeg4=scatter3(listOfJntPosLeg4(1,i),listOfJntPosLeg4(2,i),listOfJntPosLeg4(3,i),'MarkerFaceColor','b');
    currKneeJntPointLeg4=scatter3(listOfJntPosLeg4(4,i),listOfJntPosLeg4(5,i),listOfJntPosLeg4(6,i),'MarkerFaceColor','b');
    currAnkleJntPointLeg4=scatter3(listOfJntPosLeg4(7,i),listOfJntPosLeg4(8,i),listOfJntPosLeg4(9,i),'MarkerFaceColor','b');
    
    % Feet - putting this in here, might be useful once we start moving the
    % legs and feet as well
    feetPosLeg1 = scatter3(UHome(1,1),UHome(2,1), UHome(3,1),'MarkerFaceColor','k');
    feetPosLeg2 = scatter3(UHome(1,2),UHome(2,2), UHome(3,2),'MarkerFaceColor','k');
    feetPosLeg3 = scatter3(UHome(1,3),UHome(2,3), UHome(3,3),'MarkerFaceColor','k');
    feetPosLeg4 = scatter3(UHome(1,4),UHome(2,4), UHome(3,4),'MarkerFaceColor','k');

    % put points in array so easier to delete
    jntPointArr = [currHipJntPointLeg1,currKneeJntPointLeg1,currAnkleJntPointLeg1,...
        currHipJntPointLeg2,currKneeJntPointLeg2,currAnkleJntPointLeg2,...
        currHipJntPointLeg3,currKneeJntPointLeg3,currAnkleJntPointLeg3,...
        currHipJntPointLeg4,currKneeJntPointLeg4,currAnkleJntPointLeg4,...
        feetPosLeg1,feetPosLeg2, feetPosLeg3, feetPosLeg4];
    %% Plot the lines connecting the joints
    % connect hips
    hip1To2Line = plot3([listOfJntPosLeg1(1,i);listOfJntPosLeg2(1,i)],...
        [listOfJntPosLeg1(2,i);listOfJntPosLeg2(2,i)],...
        [listOfJntPosLeg1(3,i);listOfJntPosLeg2(3,i)],'r-');
    hip2To4Line = plot3([listOfJntPosLeg2(1,i);listOfJntPosLeg4(1,i)],...
        [listOfJntPosLeg2(2,i);listOfJntPosLeg4(2,i)],...
        [listOfJntPosLeg2(3,i);listOfJntPosLeg4(3,i)],'r-');
    hip3To4Line = plot3([listOfJntPosLeg3(1,i);listOfJntPosLeg4(1,i)],...
        [listOfJntPosLeg3(2,i);listOfJntPosLeg4(2,i)],...
        [listOfJntPosLeg3(3,i);listOfJntPosLeg4(3,i)],'r-');
    hip3To1Line = plot3([listOfJntPosLeg3(1,i);listOfJntPosLeg1(1,i)],...
        [listOfJntPosLeg3(2,i);listOfJntPosLeg1(2,i)],...
        [listOfJntPosLeg3(3,i);listOfJntPosLeg1(3,i)],'r-');
    
    % connect hips to knees
    hipToKneeLineLeg1 = plot3([listOfJntPosLeg1(1,i);listOfJntPosLeg1(4,i)],...
        [listOfJntPosLeg1(2,i);listOfJntPosLeg1(5,i)],...
        [listOfJntPosLeg1(3,i);listOfJntPosLeg1(6,i)],'r-');
    hipToKneeLineLeg2 = plot3([listOfJntPosLeg2(1,i);listOfJntPosLeg2(4,i)],...
        [listOfJntPosLeg2(2,i);listOfJntPosLeg2(5,i)],...
        [listOfJntPosLeg2(3,i);listOfJntPosLeg2(6,i)],'r-');
    hipToKneeLineLeg3 = plot3([listOfJntPosLeg3(1,i);listOfJntPosLeg3(4,i)],...
        [listOfJntPosLeg3(2,i);listOfJntPosLeg3(5,i)],...
        [listOfJntPosLeg3(3,i);listOfJntPosLeg3(6,i)],'r-');
    hipToKneeLineLeg4 = plot3([listOfJntPosLeg4(1,i);listOfJntPosLeg4(4,i)],...
        [listOfJntPosLeg4(2,i);listOfJntPosLeg4(5,i)],...
        [listOfJntPosLeg4(3,i);listOfJntPosLeg4(6,i)],'r-');
    
    
    % connect knees to ankles
    kneeToAnkleLineLeg1 = plot3([listOfJntPosLeg1(7,i);listOfJntPosLeg1(4,i)],...
        [listOfJntPosLeg1(8,i);listOfJntPosLeg1(5,i)],...
        [listOfJntPosLeg1(9,i);listOfJntPosLeg1(6,i)],'r-');
    kneeToAnkleLineLeg2 = plot3([listOfJntPosLeg2(7,i);listOfJntPosLeg2(4,i)],...
        [listOfJntPosLeg2(8,i);listOfJntPosLeg2(5,i)],...
        [listOfJntPosLeg2(9,i);listOfJntPosLeg2(6,i)],'r-');
    kneeToAnkleLineLeg3 = plot3([listOfJntPosLeg3(7,i);listOfJntPosLeg3(4,i)],...
        [listOfJntPosLeg3(8,i);listOfJntPosLeg3(5,i)],...
        [listOfJntPosLeg3(9,i);listOfJntPosLeg3(6,i)],'r-');
    kneeToAnkleLineLeg4 = plot3([listOfJntPosLeg4(7,i);listOfJntPosLeg4(4,i)],...
        [listOfJntPosLeg4(8,i);listOfJntPosLeg4(5,i)],...
        [listOfJntPosLeg4(9,i);listOfJntPosLeg4(6,i)],'r-');
    
    % connect ankles to feet
     ankleToFeetLineLeg1 = plot3([listOfJntPosLeg1(7,i);UHome(1,1)],...
        [listOfJntPosLeg1(8,i);UHome(2,1)],...
        [listOfJntPosLeg1(9,i);UHome(3,1)],'r-');
    ankleToFeetLineLeg2 = plot3([listOfJntPosLeg2(7,i);UHome(1,2)],...
        [listOfJntPosLeg2(8,i);UHome(2,2)],...
        [listOfJntPosLeg2(9,i);UHome(3,2)],'r-');
    ankleToFeetLineLeg3 = plot3([listOfJntPosLeg3(7,i);UHome(1,3)],...
        [listOfJntPosLeg3(8,i);UHome(2,3)],...
        [listOfJntPosLeg3(9,i);UHome(3,3)],'r-');
    ankleToFeetLineLeg4 = plot3([listOfJntPosLeg4(7,i);UHome(1,4)],...
        [listOfJntPosLeg4(8,i);UHome(2,4)],...
        [listOfJntPosLeg4(9,i);UHome(3,4)],'r-');
    
    % put in array so easier to delete afterwards
    lineArr = [hip1To2Line,hip2To4Line,hip3To4Line hip3To1Line,...
        hipToKneeLineLeg1, hipToKneeLineLeg2, hipToKneeLineLeg3,...
        hipToKneeLineLeg4,kneeToAnkleLineLeg1, kneeToAnkleLineLeg2,...
        kneeToAnkleLineLeg3,kneeToAnkleLineLeg4, ankleToFeetLineLeg1,...
        ankleToFeetLineLeg2, ankleToFeetLineLeg3, ankleToFeetLineLeg4];
    %% draw, save for giv and pause
    drawnow
    frame=getframe(fig);
    im=frame2im(frame);
    [imind,cm]=rgb2ind(im,256);
    if i==1
        imwrite(imind,cm,filename,'gif','Loopcount',inf);
    else
        imwrite(imind,cm,filename,'gif','WriteMode','append');
    end
    
    pause(0.01);
    %% Delete the previous points and lines unless its the last time point
    if i ~= length(trajectPositions)
        for k=1:length(jntPointArr)
            delete(jntPointArr(k))
        end
        for k=1:length(lineArr)
            delete(lineArr(k))
        end
    end
end

end