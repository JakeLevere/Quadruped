%% get the joint positions of the legs during the tranfer time


function[transTimeJntPosLeg1,transTimeJntPosLeg2,transTimeJntPosLeg3,transTimeJntPosLeg4] = getJntPosInTransTimeCrab(Alpha,Beta,Gamma,coxa,femur,tibia, xb_g, yb_g,zb_g)
for k = 1:length(Alpha(1,:))
    % use DH paramters to go from hip joint to the foot tip
    [~, ~, ~, kneePos_b1, anklePos_b1, footPos_b1]=legTransform(Alpha(1,k),Beta(1,k),Gamma(1,k), coxa, femur, tibia);
    [~, ~, ~, kneePos_b2, anklePos_b2, footPos_b2]=legTransform(Alpha(2,k),Beta(2,k),Gamma(2,k), coxa, femur, tibia);
    [~, ~, ~, kneePos_b3, anklePos_b3, footPos_b3]=legTransform(Alpha(3,k),Beta(3,k),Gamma(3,k), coxa, femur, tibia);
    [~, ~, ~, kneePos_b4, anklePos_b4, footPos_b4]=legTransform(Alpha(4,k),Beta(4,k),Gamma(4,k), coxa, femur, tibia);
    
    % hip Positions
    hipPos1 = [xb_g(1,k);yb_g(1); zb_g(1,k)];
    hipPos2 = [xb_g(2,k);yb_g(2); zb_g(2,k)];
    hipPos3 = [xb_g(3,k);yb_g(3); zb_g(3,k)];
    hipPos4 = [xb_g(4,k);yb_g(4); zb_g(4,k)];
    
    % Knee Positions
    %   the joint positions wrt the body. for plotting we want wrt grnd
    %   legs 1 and 3 need negative x and y values when plotting in the
    %   correct space
    kneePos1 = [kneePos_b1(1)+xb_g(1,k);-kneePos_b1(2)+yb_g(1); kneePos_b1(3)+zb_g(1,k)];
    kneePos2 = [kneePos_b2(1)+xb_g(2,k);-kneePos_b2(2)+yb_g(2); kneePos_b2(3)+zb_g(2,k)];
    kneePos3 = [-kneePos_b3(1)+xb_g(3,k);kneePos_b3(2)+yb_g(3);kneePos_b3(3)+zb_g(3,k)];
    kneePos4 = [-kneePos_b4(1)+xb_g(4,k);kneePos_b4(2)+yb_g(4); kneePos_b4(3)+zb_g(4,k)];
    % ankle Positions
    anklePos1 = [anklePos_b1(1)+xb_g(1,k);-anklePos_b1(2)+yb_g(1);anklePos_b1(3)+zb_g(1,k)];
    anklePos2 = [anklePos_b2(1)+xb_g(2,k);-anklePos_b2(2)+yb_g(2);anklePos_b2(3)+zb_g(2,k)];
    anklePos3 = [-anklePos_b3(1)+xb_g(3,k);anklePos_b3(2)+yb_g(3);anklePos_b3(3)+zb_g(3,k)];
    anklePos4 = [-anklePos_b4(1)+xb_g(4,k);anklePos_b4(2)+yb_g(4);anklePos_b4(3)+zb_g(4,k)];
    %Foot Positions
    footPos1 = [footPos_b1(1)+xb_g(1,k);-footPos_b1(2)+yb_g(1);footPos_b1(3)+zb_g(1,k)];
    footPos2 = [footPos_b2(1)+xb_g(2,k);-footPos_b2(2)+yb_g(2);footPos_b2(3)+zb_g(2,k)];
    footPos3 = [-footPos_b3(1)+xb_g(3,k);footPos_b3(2)+yb_g(3);footPos_b3(3)+zb_g(3,k)];
    footPos4 = [-footPos_b4(1)+xb_g(4,k);footPos_b4(2)+yb_g(4);footPos_b4(3)+zb_g(4,k)];
    
    % store each position in a 12xk array
    % rows 1-3 are hip, 4-6 are knee, 7-9 are ankle, 10-12 is foot pos
    % column for each time
    
    % joint positions in transfer time
    transTimeJntPosLeg1(:,k) = [hipPos1;kneePos1;anklePos1;footPos1];
    transTimeJntPosLeg2(:,k) = [hipPos2;kneePos2;anklePos2;footPos2];
    transTimeJntPosLeg3(:,k) = [hipPos3;kneePos3;anklePos3;footPos3];
    transTimeJntPosLeg4(:,k) = [hipPos4;kneePos4;anklePos4;footPos4];
    
end

end