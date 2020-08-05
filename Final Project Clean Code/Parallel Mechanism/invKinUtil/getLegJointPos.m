%% Get Leg Joint Positions Function - By Ethan Lauer
% This function obtains the hip,knee, and ankle positions for each leg for
% plotting.
%
% Units inches, radians
%
% Input: Li_vect- the set of Li vectors from the foot tip to the hip joints (3x4)
%       Li_prime_vect - the set of vectors from the foot tip to the knee joints (3x4)
%        U - matrix of U vectors for each leg (3x4 - rows are xyz, columns
%             are the legs) - origin of gnd frame to foot contact
%        theta1Home - smallest angle from the X axis to legs 1 and 2 (rad)
%        theta2Home - smallest angle from the X axis to legs 3 and 4 (rad)
%           alphai - list of alpha values for each leg (rad)
%         betai - beta values for each leg (knee angle)
%         phii - phi values for each leg (angle between the coxa and the
%                horizontal)
%       femur - length of link connecting the knee to the ankle
%
% Output: hipJntPos - positions of the hip joints in this pose (3x4) each row is
%                     xyz, each column is a leg
%         kneeJntPos- positions of the knee joints in this pose (3x4) each row is
%                     xyz, each column is a leg
%         ankleJntPos- positions of the ankle joints in this pose (3x4) each row is
%                     xyz, each column is a leg
%         footJntPos- positions of the foot joints in this pose (3x4) each row is
%                     xyz, each column is a leg

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