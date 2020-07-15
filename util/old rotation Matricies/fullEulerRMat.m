%% Full Euler Rotation Matrix Function
% This function produces the final rotation matri from the euler angles
% given a string saying what the rotation is. This can handle symbols

function [R]= fullEulerRMat(angles,rotType, isDeg)

R1= rotMatCharDeg(angles(1),rotType(1),isDeg);
R2= rotMatCharDeg(angles(2),rotType(2),isDeg);
R3= rotMatCharDeg(angles(3),rotType(3),isDeg);

R=R1*R2*R3;
end