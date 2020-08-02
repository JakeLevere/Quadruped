%% get the joint positions of the legs during the tranfer time


function[jntPosLeg1,jntPosLeg2,jntPosLeg3,jntPosLeg4] = getAllJntPositions(Alpha,Beta,Gamma,coxa,femur,tibia, xb_g, yb_g,zb_g)
intervalsTransTime = length(Alpha);

for k = 1:intervalsTransTime
    % use DH paramters to go from hip joint to the foot tip 
    [~, ~, ~, kneePos_b1, anklePos_b1, footPos_b1]=legTransform(Alpha(1,k),Beta(1,k),Gamma(1,k), coxa, femur, tibia);
    [~, ~, ~, kneePos_b2, anklePos_b2, footPos_b2]=legTransform(Alpha(2,k),Beta(2,k),Gamma(2,k), coxa, femur, tibia);
    [~, ~, ~, kneePos_b3, anklePos_b3, footPos_b3]=legTransform(Alpha(3,k),Beta(3,k),Gamma(3,k), coxa, femur, tibia);
    [~, ~, ~, kneePos_b4, anklePos_b4, footPos_b4]=legTransform(Alpha(4,k),Beta(4,k),Gamma(4,k), coxa, femur, tibia);
   
    % the joint posititions wrt the body during the transfter time for each
    % leg (3x3 mat)
    tTimeJntPos_BLeg1(:,k) = [kneePos_b1;anklePos_b1;footPos_b1];
    tTimeJntPos_BLeg2(:,k) = [kneePos_b2;anklePos_b2;footPos_b2];
    tTimeJntPos_BLeg3(:,k) = [kneePos_b3;anklePos_b3;footPos_b3];
    tTimeJntPos_BLeg4(:,k) = [kneePos_b4;anklePos_b4;footPos_b4];
end
    

fullTimeSteps = length(yb_g);
% now get it for all of the positions of the body (full time)
for k = 1:fullTimeSteps
    % hip Positions
    hipPos1 = [xb_g(1);yb_g(1,k); zb_g(1,k)];
    hipPos2 = [xb_g(2);yb_g(2,k); zb_g(2,k)];
    hipPos3 = [xb_g(3);yb_g(3,k); zb_g(3,k)];
    hipPos4 = [xb_g(4);yb_g(4,k); zb_g(4,k)];
    
    
    col = 1+mod(k-1,intervalsTransTime);
    % Knee Positions 
    %   the joint positions wrt the body. for plotting we want wrt grnd
    %   legs 1 and 3 need negative x and y values when plotting in the
    %   correct space
    kneePos1 = [-tTimeJntPos_BLeg1(1,col)+xb_g(1);-tTimeJntPos_BLeg1(2,col)+yb_g(1,k); tTimeJntPos_BLeg1(3,col)+zb_g(1,k)];
    kneePos2 = tTimeJntPos_BLeg2(1:3,col)+[xb_g(2);yb_g(2,k);zb_g(2,k)];
    kneePos3 = [-tTimeJntPos_BLeg3(1,col)+xb_g(3);-tTimeJntPos_BLeg3(2,col)+yb_g(3,k);tTimeJntPos_BLeg3(3,col)+zb_g(3,k)];
    kneePos4 = tTimeJntPos_BLeg4(1:3,col)+[xb_g(4);yb_g(4,k);zb_g(4,k)];
    
    % ankle Positions
    anklePos1 = [-tTimeJntPos_BLeg1(4,col)+xb_g(1);-tTimeJntPos_BLeg1(5,col)+yb_g(1,k);tTimeJntPos_BLeg1(6,col)+zb_g(1,k)];
    anklePos2 = tTimeJntPos_BLeg2(4:6,col)+[xb_g(2);yb_g(2,k);zb_g(2,k)];
    anklePos3 = [-tTimeJntPos_BLeg3(4,col)+xb_g(3);-tTimeJntPos_BLeg3(5,col)+yb_g(3,k);tTimeJntPos_BLeg3(6,col)+zb_g(3,k)];
    anklePos4 = tTimeJntPos_BLeg4(4:6,col)+[xb_g(4);yb_g(4,k);zb_g(4,k)];
    
    %Foot Positions
    footPos1 = [-tTimeJntPos_BLeg1(7,col)+xb_g(1);-tTimeJntPos_BLeg1(8,col)+yb_g(1,k);tTimeJntPos_BLeg1(9,col)+zb_g(1,k)];
    footPos2 = tTimeJntPos_BLeg2(7:9,col)+[xb_g(2);yb_g(2,k);zb_g(2,k)];
    footPos3 = [-tTimeJntPos_BLeg3(7,col)+xb_g(3);-tTimeJntPos_BLeg3(8,col)+yb_g(3,k);tTimeJntPos_BLeg3(9,col)+zb_g(3,k)];
    footPos4 = tTimeJntPos_BLeg4(7:9,col)+[xb_g(4);yb_g(4,k);zb_g(4,k)];
    
    
    % store each position in a 12xk array
    % rows 1-3 are hip, 4-6 are knee, 7-9 are ankle, 10-12 is foot pos
    % column for each time
    
    % joint positions in transfer time
    jntPosLeg1(:,k) = [hipPos1;kneePos1;anklePos1;footPos1];
    jntPosLeg2(:,k) = [hipPos2;kneePos2;anklePos2;footPos2];
    jntPosLeg3(:,k) = [hipPos3;kneePos3;anklePos3;footPos3];  
    jntPosLeg4(:,k) = [hipPos4;kneePos4;anklePos4;footPos4];
    
end

end