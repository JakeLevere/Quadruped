%% Joint Positions for a Single Cycle using the Kinematic Phase
% This function produces the list of joint positions for each leg in a
% single cycle using the kinematic phase for plotting. This is only for a
% duty factor of 0.75.
%
% Input: p - the kinematic phase for each leg (1x4)
%         tTJntPosLeg1 - list of leg 1's joint positions during the
%                       transfer phase
%         tTJntPosLeg2 - list of leg 2's joint positions during the
%                       transfer phase
%         tTJntPosLeg3 - list of leg 3's joint positions during the
%                       transfer phase
%         tTJntPosLeg4 - list of leg 4's joint positions during the
%                       transfer phase
%
% Output: cycleTimeJntPosLeg1 - list of leg 1's joint positions during the
%                       full cycle
%         cycleTimeJntPosLeg2 - list of leg 2's joint positions during the
%                       full cycle
%         cycleTimeJntPosLeg3 - list of leg 3's joint positions during the
%                       full cycle
%         cycleTimeJntPosLeg4 - list of leg 4's joint positions during the
%                       full cycle
%         time - time matrix


function [cycleTimeJntPosLeg1,cycleTimeJntPosLeg2,cycleTimeJntPosLeg3,cycleTimeJntPosLeg4, time] = cyclePosWithPhase(p, tTJntPosLeg1,tTJntPosLeg2,tTJntPosLeg3,tTJntPosLeg4)
time = linspace(0,1,16);
for i = 1:length(time)
    t = time(i); % current time
    
    if t>=0 && t<p(4) %leg 4 moves
        cycleTimeJntPosLeg4(:,i)= tTJntPosLeg4(:,i);
        cycleTimeJntPosLeg2(:,i)= tTJntPosLeg2(:,1);
        cycleTimeJntPosLeg3(:,i)= tTJntPosLeg3(:,1);
        cycleTimeJntPosLeg1(:,i)= tTJntPosLeg1(:,1);
    end
    if t>p(4) && t<p(2) %leg 2 moves
        cycleTimeJntPosLeg2(:,i)= tTJntPosLeg2(:,i-4);
        cycleTimeJntPosLeg4(:,i)= tTJntPosLeg4(:,end);
        cycleTimeJntPosLeg3(:,i)= tTJntPosLeg3(:,1);
        cycleTimeJntPosLeg1(:,i)= tTJntPosLeg1(:,1);
    end
    if t>p(2) && t<p(3) % leg 3 moves
        cycleTimeJntPosLeg3(:,i)= tTJntPosLeg3(:,i-8);
        cycleTimeJntPosLeg4(:,i)= tTJntPosLeg4(:,end);
        cycleTimeJntPosLeg2(:,i)= tTJntPosLeg2(:,end);
        cycleTimeJntPosLeg1(:,i)= tTJntPosLeg1(:,1);
    end
    if t>p(3) && t<=time(end)% leg 1 moves
        cycleTimeJntPosLeg1(:,i)= tTJntPosLeg1(:,i-11);
        cycleTimeJntPosLeg2(:,i)= tTJntPosLeg2(:,end);
        cycleTimeJntPosLeg4(:,i)= tTJntPosLeg4(:,end);
        cycleTimeJntPosLeg3(:,i)= tTJntPosLeg3(:,end);
    end
end

end