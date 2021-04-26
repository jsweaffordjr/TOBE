function Amatrix = Amat(jointangle,jointoffset,linklength,twistangle)

% This function computes the Jacobian with respect to the left stance foot,
% given the current joint angles in degrees

d = jointoffset;
a = linklength;

CT = cosd(jointangle);
ST = sind(jointangle);
CA = cosd(twistangle);
SA = sind(twistangle);


Amatrix = [CT, -ST*CA, ST*SA, a*CT; 
           ST, CT*CA, -CT*SA, a*ST; 
           0, SA, CA, d; 
           0, 0, 0, 1];

end

