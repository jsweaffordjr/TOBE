function Amatrix = Amat2(jointangle,jointoffset,linklength,twistangle)

% This function computes the A transformation matrix between two successive
% frames, given the joint angle, joint offset, link length and twist angle
% between the two frames. It is assumed that the angles are given in
% degrees. Alternate method of transformation: twist angle first, then
% link length, joint offset, and joint angle

d = jointoffset;
a = linklength;

CT = cosd(jointangle);
ST = sind(jointangle);
CA = cosd(twistangle);
SA = sind(twistangle);


Amatrix = [CT, -ST, 0, a; 
    ST*CA, CT*CA, -SA, -d*SA; 
    ST*SA, CT*SA, CA, d*CA; 
    0, 0, 0, 1];

end

