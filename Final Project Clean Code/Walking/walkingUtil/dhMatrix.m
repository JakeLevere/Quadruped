%% DH Matrix Function -by Ethan Lauer
% This calculates the DH matrix based on the dh parameters provided
%
% Input: q - rotation about z axis
%         d - translation from z to z
%         a  - translation from x to x
%         alpha - rotation from x to x
%
% Output: dh - dh Matrix
function [dh]=dhMatrix(q,d,a,alpha)
dh=[cos(q),-sin(q)*cos(alpha),sin(q)*sin(alpha),a*cos(q);
    sin(q),cos(q)*cos(alpha),-cos(q)*sin(alpha),a*sin(q);
    0,sin(alpha),cos(alpha),d;
    0,0,0,1];   
end