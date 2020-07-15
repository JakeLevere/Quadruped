%% getLegJointPos Function - By Ethan Lauer
% This function obtains the hip,knee, and ankle positions for each leg for
% plotting.
% Given: Li_prime_vect = the set of vectors from the foot tip to the knee
% joints, Li_prime_mag = the magnitude of the vector, U vectors from origin
% to foot tips, home theta values, alpha and beta values, femur length,
% boolean true if using degrees, false if radians.
% Result: the hip,knee, and ankle positions in 3D space.

function[hipJntPos,kneeJntPos,ankleJntPos] = getLegJointPos(Li_vect, Li_prime_vect, U, theta1Home,theta2Home, alphai, betai, phii,femur, isDeg)
hipJntPos = U+Li_vect;
kneeJntPos = U+Li_prime_vect;
for i=1:length(U)
    ankleAngle = betai(i)+phii(i);
    [sinAnkleAngle, cosAnkleAngle] = sinCosDegRad(ankleAngle,isDeg);
    hyp = femur*cosAnkleAngle; % make into radians if needed (create function for plotting these positions)
    ankleZ = femur*sinAnkleAngle;
    switch i
        case 1
            xyangle = theta1Home-alphai(i);
            [sinxyangle, cosxyangle] = sinCosDegRad(xyangle,isDeg);
            ankleX = -hyp*cosxyangle;
            ankleY = hyp*sinxyangle;
        case 2
            xyangle = theta1Home+alphai(i);
            [sinxyangle, cosxyangle] = sinCosDegRad(xyangle,isDeg);
            ankleX = hyp*cosxyangle;
            ankleY = hyp*sinxyangle;
        case 3
            xyangle = theta2Home+alphai(i);
            [sinxyangle, cosxyangle] = sinCosDegRad(xyangle,isDeg);
            
            ankleX = -hyp*cosxyangle;
            ankleY = -hyp*sinxyangle;
        case 4
            xyangle = theta2Home-alphai(i);
            [sinxyangle, cosxyangle] = sinCosDegRad(xyangle,isDeg);
            ankleX = hyp*cosxyangle;
            ankleY = -hyp*sinxyangle;
    end
    ankleJntPos(:,i) = kneeJntPos(:,i) +[ankleX;ankleY;ankleZ];
end
end