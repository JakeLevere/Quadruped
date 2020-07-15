%% Rotation Matrix Structure 
% This Function creates a rotation matrix of a given angle in radians or
% degrees. This also handles symbolic variables

function[rot] = rotMatStruct(angle)

rot.x=[1,0,0;
        0,cos(angle),-sin(angle);
        0,sin(angle),cos(angle)];
    
rot.xd=[1,0,0;
        0,cosd(angle),-sind(angle);
        0,sind(angle),cosd(angle)];
    
rot.y=[cos(angle),0,sin(angle);
        0,1,0;
        -sin(angle),0,cos(angle)];
    
rot.yd=[cos(angle),0,sin(angle);
        0,1,0;
        -sin(angle),0,cos(angle)];
    
    
rot.z=[cos(angle),-sin(angle),0;
        sin(angle),cos(angle),0
        0,0,1];
    
rot.zd=[cosd(angle),-sind(angle),0;
        sind(angle),cosd(angle),0
        0,0,1];
end