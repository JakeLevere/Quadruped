%% Law of cosines function
% This function returns the angle in degrees or radians given the lengths
% of a triangle.
function [angle] = lawOfCos(a,b,c, desAngle, isDeg)
a2 = a^2;
b2 = b^2;
c2 = c^2;

switch desAngle
    case 'A'
        if isDeg
            angle = acosd((b2+c2-a2)/(2*b*c));
        else
            angle = acos((b2+c2-a2)/(2*b*c));
        end
        
    case 'B'
        if isDeg
            angle = acosd((c2+a2-b2)/(2*c*a));
        else
            angle = acos((c2+a2-b2)/(2*c*a));
        end
        
    case 'C'
        if isDeg
            angle = acosd((a2+b2-c2)/(2*a*b));
        else
            angle = acos((a2+b2-c2)/(2*a*b));
        end     
end
end