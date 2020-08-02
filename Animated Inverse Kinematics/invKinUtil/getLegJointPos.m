%% getLegJointPos Function - By Ethan Lauer
% This function obtains the hip,knee, and ankle positions for each leg for
% plotting.
% Given: Li_prime_vect = the set of vectors from the foot tip to the knee
% joints, Li_prime_mag = the magnitude of the vector, U vectors from origin
% to foot tips, home theta values, alpha and beta values, femur length,
% boolean true if using degrees, false if radians.
% Result: the hip,knee, and ankle positions in 3D space.

function[hipJntPos,kneeJntPos,ankleJntPos, footJntPos] = getLegJointPos(Li_vect, Li_prime_vect, U, theta1Home,theta2Home, alphai, betai, phii,femur)
hipJntPos = U+Li_vect;
kneeJntPos = U+Li_prime_vect;
footJntPos = U;
for i=1:length(U)
    ankleAngle = betai(i)+phii(i);
    hyp = femur*cos(ankleAngle); % make into radians if needed (create function for plotting these positions)
    ankleZ = femur*sin(ankleAngle);
    switch i
        case 1
            xyangle = theta1Home-alphai(i);
            ankleX = -hyp*cos(xyangle);
            ankleY = hyp*sin(xyangle);
        case 2
            xyangle = theta1Home+alphai(i);
            ankleX = hyp*cos(xyangle);
            ankleY = hyp*sin(xyangle);
        case 3
            xyangle = theta2Home+alphai(i);            
            ankleX = -hyp*cos(xyangle);
            ankleY = -hyp*sin(xyangle);
        case 4
            xyangle = theta2Home-alphai(i);
            ankleX = hyp*cos(xyangle);
            ankleY = -hyp*sin(xyangle);
    end
    ankleJntPos(:,i) = kneeJntPos(:,i) +[ankleX;ankleY;ankleZ];
end
end