%% Gait Generation - Ethan Lauer
% This generates the kinematic phase for the four legs of the robot.
%
% Input: beta - duty factor between 0 and 1 (0.75 used)
%
% Output: p - kinematic phase (1x4)

function[p] = gaitGen(beta)
p(1)=0; % Kinematic phase of leg 1
p(2)=p(1)+1/2; % Kinematic phase of leg 2
p(3)=p(1)+beta; % Kinematic phase of leg 3
p(4)=p(2)+beta; % Kinematic phase of leg 4

j=1;
for j=1:4
    for i=1:4
        if p(i)>=1
            p(i)=p(i)-1;
        end
    end
    j=j+1;
end
end