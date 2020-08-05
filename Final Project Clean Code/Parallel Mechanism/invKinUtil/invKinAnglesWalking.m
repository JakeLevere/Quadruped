%% Inverse Kinematics Walking Angles - By Ethan Lauer
% This function finds the rest of the angles of the joints needed for
% inverse kinematics for a walking robot.
%
% Units: radians, inches
%
% Input: Li_prime_vect - the set of vectors from the foot tip to the knee joints (3x4)
%       Li_prime_mag  - the distance from the foot tip to the knee joints (1x4)
%       Li_vect- the set of Li vectors from the foot tip to the hip joints (3x4)
%       coxa - length of link connecting the hip to the knee
%       femur - length of link connecting the knee to the ankle
%       tibia - length of link connecting ankle to the foot tip
%
% Output: betai - beta values for each leg (knee angle)
%         gammai - gamma values for each leg (ankle angle)
%         lamdai - lamda values for each leg (angle from Liprime and tibia)
%         phii - phi values for each leg (angle between the coxa and the
%                horizontal)
%         rhoi - rho angles for each leg (angle between Li and horizontal)

function [betai,gammai,lamdai,phii,rhoi] = invKinAnglesWalking(Li_prime_vect, Li_prime_mag,Li_vect,coxa,femur,tibia)
numLegs = length(Li_prime_vect);
betai = zeros(1,numLegs);
lamdai = zeros(1,numLegs);
gammai = zeros(1,numLegs);
phii = zeros(1,numLegs);
rhoi = zeros(1,numLegs);
% for each leg, find values for all the angles
for i = 1:numLegs
    deltah = Li_prime_vect(3,i)-Li_vect(3,i); % hi_prime - hi
    
    rho_denom = sqrt((Li_prime_vect(1,i)^2)+(Li_prime_vect(2,i)^2));
    
    beta_num = femur^2+Li_prime_mag(i)^2-tibia^2; % not sure if this is magnitute of Liprime or should be like rho_denom;
    beta_denom = 2*femur*Li_prime_mag(i);
    
    gamma_num = femur^2+tibia^2-Li_prime_mag(i)^2;
    gamma_denom = 2*femur*tibia;
    
    lamda_num = Li_prime_mag(i)^2+tibia^2-femur^2;
    lamda_denom = 2*Li_prime_mag(i)*tibia;
    
    phii(i) = asin(deltah/coxa);
    rhoi(i) = atan( Li_prime_vect(3,i)/rho_denom);
    betai(i) = acos(beta_num/beta_denom)-(rhoi(i)+phii(i));
    gammai(i) = pi-acos(gamma_num/gamma_denom);
    lamdai(i) = acos(lamda_num/lamda_denom);
end
if any(gammai<0)
    error('Gamma cannot be less than 0');
end

% check if there is an imaginary/complex number
allAngles =  [phii;rhoi;betai;gammai;lamdai];
if ~isreal(allAngles)
    error('Angles have complex numbers. Try different position. Might be close to a Singularity');
end
end