function q = LegInterface(LegExtension,phi_leg,phi_foot)
% based on Missura PhD thesis, 2015 and 
% Capture Steps paper by Missura, Bennewitz, Behnke, 2020
% this function takes as inputs:
% 1. leg extension, eta = [eta_R;eta_L] [between 0 (fully extended) and 1 {fully retracted)]
% 2. leg angle, phi_leg = [phi_roll_R, phi_roll_L; phi_pitch_R, phi_pitch_L; phi_yaw_R, phi_yaw_L]_leg
% 3. foot angle, phi_foot = [phi_roll_R, phi_roll_L; phi_pitch_R, phi_pitch_L]_foot

% and outputs joint angle vector q,
% where q = [LfrontalAnkle;LsagAnkle;LKnee;LsagHip;LfrontalHip;LyawHip;
%            RyawHip;RfrontalHip;RsagHip;RKnee;RsagAnkle;RfrontalAnkle]

% parameters:
eta_R = LegExtension(1);
eta_L = LegExtension(2);

% eta = 1 denotes fully retracted leg, which should not be an issue, but
% eta = 0 denotes fully extended leg and cannot be negative. 
% The following lines ensure that this does not happen.
if eta_R < 0
    eta_R = 0;
end
if eta_L < 0
    eta_L = 0;
end

% angles computed from leg extension parameters
zeta_R = -acos(1-eta_R); 
zeta_L = -acos(1-eta_L);

% leg yaw values are taken directly from 'phi_leg'
Rlegyaw = phi_leg(3,1);
Llegyaw = phi_leg(3,2);

% compute alternate pitch and leg angles
Rlegprime = [cos(-Rlegyaw),-sin(-Rlegyaw);sin(-Rlegyaw),cos(-Rlegyaw)]*[phi_leg(2,1);phi_leg(1,1)]; 
Rlegroll = Rlegprime(2);
Rlegpitch = Rlegprime(1);
Llegprime = [cos(-Llegyaw),-sin(-Llegyaw);sin(-Llegyaw),cos(-Llegyaw)]*[phi_leg(2,2);phi_leg(1,2)]; 
Llegroll = Llegprime(2);
Llegpitch = Llegprime(1);

% assignment of joint angles:
LfrontalAnkle = phi_foot(1,2) + Llegroll; 
LsagAnkle = (phi_foot(2,2) - Llegpitch - zeta_L);
LKnee = 2*zeta_L;
LsagHip = (Llegpitch - zeta_L);
LfrontalHip = Llegroll;
LyawHip = Llegyaw;
RyawHip = Rlegyaw;
RfrontalHip = Rlegroll;
RsagHip = (Rlegpitch - zeta_R);
RKnee = 2*zeta_R;
RsagAnkle = (phi_foot(2,1) - Rlegpitch - zeta_R);
RfrontalAnkle = phi_foot(1,1) - Rlegroll;

q = [LfrontalAnkle;LsagAnkle;LKnee;LsagHip;LfrontalHip;LyawHip;
     RyawHip;RfrontalHip;RsagHip;RKnee;RsagAnkle;RfrontalAnkle];

end

