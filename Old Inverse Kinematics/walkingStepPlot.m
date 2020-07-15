function [] = walkingStepPlot(currPose,U, hipJntPos,kneeJntPos,ankleJntPos)
plot3(0,0,0,'*k') % origin
hold on
plot3(currPose(1),currPose(2),currPose(3),'*g') % goal pose
hold on
for i =1:4
    % Plot joint positions
    plot3(U(1,i),U(2,i),U(3,i),'*b');
    hold on
    plot3(hipJntPos(1,i),hipJntPos(2,i),hipJntPos(3,i),'*b-');
    hold on
    plot3(kneeJntPos(1,i),kneeJntPos(2,i),kneeJntPos(3,i),'*b-');
    hold on
    plot3(ankleJntPos(1,i),ankleJntPos(2,i),ankleJntPos(3,i),'*b-');
    hold on
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

% Connect the main body of the robot with lines
for i=1:2
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

end