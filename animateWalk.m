%% Animate Walk - By Ethan Lauer
% this function plots the animation of the walking robot and saves it as a gif.
% Front legs are light blue, back are dark blue
% Given:list of joint positions for each leg, title and file name string
function [] =animateWalk(timeMat,listOfJntPosLeg1,listOfJntPosLeg2,listOfJntPosLeg3,listOfJntPosLeg4, Title, fileName, limits)
fig = figure('Name', Title);
grid on
xlim(limits(1,:))
ylim(limits(2,:))
zlim(limits(3,:))

% xlim([-8,8])
% ylim([-8,16])
% zlim([0,8])
hold on
xlabel('X Position (in)');
zlabel('Z Position (in)');
ylabel('Y Position (in)');
title(Title)
filename= fileName;
view(70,45);
% view(0,90);


for i=1:length(timeMat)
    
    %% Plot joint and feet points in space
    % Front legs are light blue, back are dark blue
    
    % Leg 1
    currHipJntPointLeg1=scatter3(listOfJntPosLeg1(1,i),listOfJntPosLeg1(2,i),listOfJntPosLeg1(3,i),'MarkerFaceColor','r');
    currKneeJntPointLeg1=scatter3(listOfJntPosLeg1(4,i),listOfJntPosLeg1(5,i),listOfJntPosLeg1(6,i),'MarkerFaceColor','r');
    currAnkleJntPointLeg1=scatter3(listOfJntPosLeg1(7,i),listOfJntPosLeg1(8,i),listOfJntPosLeg1(9,i),'MarkerFaceColor','r');
    currFootJntPointLeg1=scatter3(listOfJntPosLeg1(10,i),listOfJntPosLeg1(11,i),listOfJntPosLeg1(12,i),'MarkerFaceColor','r');
    
    % Leg 2
    currHipJntPointLeg2=scatter3(listOfJntPosLeg2(1,i),listOfJntPosLeg2(2,i),listOfJntPosLeg2(3,i),'MarkerFaceColor','g');
    currKneeJntPointLeg2=scatter3(listOfJntPosLeg2(4,i),listOfJntPosLeg2(5,i),listOfJntPosLeg2(6,i),'MarkerFaceColor','g');
    currAnkleJntPointLeg2=scatter3(listOfJntPosLeg2(7,i),listOfJntPosLeg2(8,i),listOfJntPosLeg2(9,i),'MarkerFaceColor','g');
    currFootJntPointLeg2=scatter3(listOfJntPosLeg2(10,i),listOfJntPosLeg2(11,i),listOfJntPosLeg2(12,i),'MarkerFaceColor','g');
    
    % Leg 3
    currHipJntPointLeg3=scatter3(listOfJntPosLeg3(1,i),listOfJntPosLeg3(2,i),listOfJntPosLeg3(3,i),'MarkerFaceColor','b');
    currKneeJntPointLeg3=scatter3(listOfJntPosLeg3(4,i),listOfJntPosLeg3(5,i),listOfJntPosLeg3(6,i),'MarkerFaceColor','b');
    currAnkleJntPointLeg3=scatter3(listOfJntPosLeg3(7,i),listOfJntPosLeg3(8,i),listOfJntPosLeg3(9,i),'MarkerFaceColor','b');
    currFootJntPointLeg3=scatter3(listOfJntPosLeg3(10,i),listOfJntPosLeg3(11,i),listOfJntPosLeg3(12,i),'MarkerFaceColor','b');
    
    % Leg 4
    currHipJntPointLeg4=scatter3(listOfJntPosLeg4(1,i),listOfJntPosLeg4(2,i),listOfJntPosLeg4(3,i),'MarkerFaceColor','c');
    currKneeJntPointLeg4=scatter3(listOfJntPosLeg4(4,i),listOfJntPosLeg4(5,i),listOfJntPosLeg4(6,i),'MarkerFaceColor','c');
    currAnkleJntPointLeg4=scatter3(listOfJntPosLeg4(7,i),listOfJntPosLeg4(8,i),listOfJntPosLeg4(9,i),'MarkerFaceColor','c');
    currFootJntPointLeg4=scatter3(listOfJntPosLeg4(10,i),listOfJntPosLeg4(11,i),listOfJntPosLeg4(12,i),'MarkerFaceColor','c');
    
    
    % put points in array so easier to delete
    jntPointArr = [currHipJntPointLeg1,currKneeJntPointLeg1,currAnkleJntPointLeg1,...
        currHipJntPointLeg2,currKneeJntPointLeg2,currAnkleJntPointLeg2,...
        currHipJntPointLeg3,currKneeJntPointLeg3,currAnkleJntPointLeg3,...
        currHipJntPointLeg4,currKneeJntPointLeg4,currAnkleJntPointLeg4,...
        currFootJntPointLeg1,currFootJntPointLeg2, currFootJntPointLeg3, currFootJntPointLeg4];
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
    ankleToFeetLineLeg1 = plot3([listOfJntPosLeg1(7,i);listOfJntPosLeg1(10,i)],...
        [listOfJntPosLeg1(8,i);listOfJntPosLeg1(11,i)],...
        [listOfJntPosLeg1(9,i);listOfJntPosLeg1(12,i)],'r-');
    ankleToFeetLineLeg2 = plot3([listOfJntPosLeg2(7,i);listOfJntPosLeg2(10,i)],...
        [listOfJntPosLeg2(8,i);listOfJntPosLeg2(11,i)],...
        [listOfJntPosLeg2(9,i);listOfJntPosLeg2(12,i)],'r-');
   ankleToFeetLineLeg3 = plot3([listOfJntPosLeg3(7,i);listOfJntPosLeg3(10,i)],...
        [listOfJntPosLeg3(8,i);listOfJntPosLeg3(11,i)],...
        [listOfJntPosLeg3(9,i);listOfJntPosLeg3(12,i)],'r-');
    ankleToFeetLineLeg4 = plot3([listOfJntPosLeg4(7,i);listOfJntPosLeg4(10,i)],...
        [listOfJntPosLeg4(8,i);listOfJntPosLeg4(11,i)],...
        [listOfJntPosLeg4(9,i);listOfJntPosLeg4(12,i)],'r-');
    
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
    if i ~= length(timeMat)
        for k=1:length(jntPointArr)
            delete(jntPointArr(k))
        end
        for k=1:length(lineArr)
            delete(lineArr(k))
        end
    end
end
end