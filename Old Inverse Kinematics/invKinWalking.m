function[alphai,betai,lamdai,gammai,phii,rhoi,Li_vect,Li_prime_vect,si2] = invKinWalking(pose,si1,U,coxa,femur,tibia,rotType,isDeg)
% find Li vector (format: rows are x,y,z, colums are legs)
[Li_vect,Li_mag] = invKin(pose,si1,U,rotType,isDeg);

% Get si2 and alpha values
[si2,alphai] = getSi2Alphai(si1,coxa,Li_vect, isDeg);

% find Liprime vector (format: rows are x,y,z, colums are legs)
[Li_prime_vect,Li_prime_mag] = invKin(pose,si2,U,rotType,isDeg);

% get the angles for beta, gamma and all others
[betai,gammai,lamdai,phii,rhoi] = invKinAnglesWalking(Li_prime_vect, Li_prime_mag,Li_vect,coxa,femur,tibia, isDeg);
end