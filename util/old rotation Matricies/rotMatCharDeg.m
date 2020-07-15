%% Single Rotation Matrix 
% This creates a rotaiton matrix for a given angle about a certain axis
% based on string input as well as if it is in degrees or not.

function [Ri] = rotMatCharDeg(angle,char,isDeg)
r = rotMatStruct(angle);
if isDeg
    switch char
        case 'X'
            Ri=r.xd;
        case 'Y'
            Ri=r.yd;
        case 'Z'
            Ri=r.zd;
    end
    
else
    switch char
          case 'X'
            Ri=r.x;
        case 'Y'
            Ri=r.y;
        case 'Z'
            Ri=r.z;
    end
end
end