function [eta,phi_leg,phi_foot] = LegMotion(A,mu)
% largely based on Missura PhD thesis, 2015
% inputs: swing amplitude A; motion phase, mu; 
% outputs: leg extension, eta; leg angle, phi_leg; foot angle, phi_foot

% configuration parameters:
K1 = 0.35; % home position leg extension
K2 = -0.15; % home position leg roll angle
K3 = -0.45; % home position leg pitch angle
K4 = 0.05; % home position foot roll angle
K5 = -0.12; % home position foot pitch angle

prestance = 0.1; % stance hip position (from home) just before opposite leg swings
stmax = -0.05; % max stance hip displacement during opposite leg swing
preswing = 0.1; % swing hip position (from home) just before swing
swmax = -0.0; % max swing hip displacement during swing

prepro = -0.2; % stance ankle angle just before pronation sequence
presup = 0.1; % swing ankle angle just before supination sequence
pron = 0; % max angle change during pronation of non-swing foot

K_mu2 = 2; % mid-swing time
K13 = 0.75; % sagittal swing amplitude

% parameters that shouldn't need as much tuning:
K6 = 0.04; % ground push constant
K7 = 0; % ground push intensifier
K8 = 0.06; % step height constant
K9 = 0.03; % step height intensifier
K_mu0 = 0.6; % swing start time
K_mu1 = 3; % swing stop time
sup = 0; % max angle change during swing phase supination
K10 = 0.12; % lateral swing amplitude
K11 = 0.1; % lateral swing amplitude offset
K12 = 0.01; % turning lateral swing amplitude offset
K14 = 0.4; % rotational swing amplitude
K15 = 0.05; % rotational swing amplitude offset

% 1. Home/halt position
eta_home = K1;
phi_Rleg_home = [K2; K3; 0];
phi_Lleg_home = [-K2; K3; 0];
phi_Rfoot_home = [K4; K5];
phi_Lfoot_home = [K4; K5];

% 2. Leg lifting
if mu > K_mu0 && mu < K_mu1 % right swing phase
    if mu < K_mu2 % during leg lifting 
        eta_Rleg_lift = sin(0.5*pi*(mu - K_mu0)/(K_mu2 - K_mu0))*(K8 + K9*max(abs(A)));
    else % during leg lowering
        eta_Rleg_lift = sin(0.5*pi*(1+(mu - K_mu2)/(K_mu1 - K_mu2)))*(K8 + K9*max(abs(A))); 
    end
else % right stance phase
   if mu >= K_mu1
       nu_R1 = -pi + pi*(mu - K_mu1)/(2*pi + K_mu0 - K_mu1);
   else
       nu_R1 = -pi + pi*(mu - K_mu1 + 2*pi)/(2*pi + K_mu0 - K_mu1);
   end 
   eta_Rleg_lift = sin(0.5*nu_R1)*(K6 + K7*max(abs(A)));
end

if mu > K_mu0 - pi && mu < K_mu1 - pi % left swing phase
    if mu < K_mu2 - pi % during leg lifting
        eta_Lleg_lift = sin(0.5*pi*(mu - K_mu0 + pi)/(K_mu2 - K_mu0))*(K8 + K9*max(abs(A)));
    else % during leg lowering
        eta_Lleg_lift = sin(0.5*pi*(1+(mu - K_mu2 + pi)/(K_mu1 - K_mu2)))*(K8 + K9*max(abs(A))); 
    end
else % left stance phase
   if mu >= K_mu1 - pi
       nu_L1 = -pi + pi*(mu - K_mu1 + pi)/(2*pi + K_mu0 - K_mu1);
   else
       nu_L1 = -pi + pi*(mu - K_mu1 + 3*pi)/(2*pi + K_mu0 - K_mu1);
   end
   eta_Lleg_lift = sin(0.5*nu_L1)*(K6 + K7*max(abs(A)));    
end

% 3. Leg swing
if mu < K_mu0
    zeta_R = ((2*(mu + 2*pi - K_mu1))/(2*pi - K_mu1 + K_mu0)) - 1;
elseif mu < K_mu1
    zeta_R = cos((pi*(mu - K_mu0))/(K_mu1 - K_mu0));
else
    zeta_R = ((2*(mu - K_mu1))/(2*pi - K_mu1 + K_mu0)) - 1;
end

RLegSwingRoll = -zeta_R*A(1)*K10 - max(abs(A(1))*K11,abs(A(3)*K12));
RLegSwingPitch = zeta_R*A(2)*K13;
RLegSwingYaw = zeta_R*A(3)*K14 - abs(A(3))*K15;
RLegSwing = [RLegSwingRoll; RLegSwingPitch; RLegSwingYaw];

if mu < K_mu0 - pi
    zeta_L = ((2*(mu + 3*pi - K_mu1))/(2*pi - K_mu1 + K_mu0)) - 1;
elseif mu < K_mu1 - pi
    zeta_L = cos((pi*(mu + pi - K_mu0))/(K_mu1 - K_mu0));
else
    zeta_L = ((2*(mu + pi - K_mu1))/(2*pi - K_mu1 + K_mu0)) - 1;
end

LLegSwingRoll = -zeta_L*A(1)*K10 + max(abs(A(1))*K11,abs(A(3)*K12));
LLegSwingPitch = zeta_L*A(2)*K13;
LLegSwingYaw = zeta_L*A(3)*K14 + abs(A(3))*K15;
LLegSwing = [LLegSwingRoll; LLegSwingPitch; LLegSwingYaw];

% 4. Hip swing and Ankle pronation/supination
if mu >= K_mu0 - pi && mu < K_mu1 - pi % left swing phase
    hip_R = prestance + stmax*sin(pi*(mu - K_mu0 + pi)/(K_mu1 - K_mu0));
    hip_L = preswing + swmax*sin(pi*(mu - K_mu0 + pi)/(K_mu1 - K_mu0));
    ank_R = prepro - pron*sin(pi*(mu - K_mu0 + pi)/(K_mu1 - K_mu0));
    ank_L = presup + sup*sin(pi*(mu - K_mu0 + pi)/(K_mu1 - K_mu0));
elseif mu >= K_mu1 - pi && mu < K_mu0 % double support leading to right swing 
    hip_R = prestance + (-preswing - prestance)*sin(0.5*pi*(mu - K_mu1 + pi)/(K_mu0 - K_mu1 + pi));
    hip_L = preswing + (-prestance - preswing)*sin(0.5*pi*(mu - K_mu1 + pi)/(K_mu0 - K_mu1 + pi));
    ank_R = prepro + (presup - prepro)*(mu - K_mu1 + pi)/(K_mu0 - K_mu1 + pi);
    ank_L = presup + (prepro - presup)*(mu - K_mu1 + pi)/(K_mu0 - K_mu1 + pi);
elseif mu >= K_mu0 && mu < K_mu1 % right swing phase 
    hip_R = -preswing - swmax*sin(pi*(mu - K_mu0)/(K_mu1 - K_mu0));
    hip_L = -prestance - stmax*sin(pi*(mu - K_mu0)/(K_mu1 - K_mu0));
    ank_R = presup + sup*sin(pi*(mu - K_mu0)/(K_mu1 - K_mu0));
    ank_L = prepro - pron*sin(pi*(mu - K_mu0)/(K_mu1 - K_mu0));
elseif mu >= K_mu1 % double support leading to left swing, part 1
    hip_R = -preswing + (prestance + preswing)*sin(0.5*pi*(mu - K_mu1)/(K_mu0 - K_mu1 + pi));
    hip_L = -prestance + (preswing + prestance)*sin(0.5*pi*(mu - K_mu1)/(K_mu0 - K_mu1 + pi));
    ank_R = presup + (prepro - presup)*(mu - K_mu1)/(K_mu0 - K_mu1 + pi);
    ank_L = prepro + (presup - prepro)*(mu - K_mu1)/(K_mu0 - K_mu1 + pi);
else % double support leading to left swing, part 2 (mu < K_mu0 - pi)
    hip_R = -preswing + (prestance + preswing)*sin(0.5*pi*(mu + 2*pi - K_mu1)/(K_mu0 - K_mu1 + pi));
    hip_L = -prestance + (preswing + prestance)*sin(0.5*pi*(mu + 2*pi - K_mu1)/(K_mu0 - K_mu1 + pi));
    ank_R = presup + (prepro - presup)*(mu + 2*pi - K_mu1)/(K_mu0 - K_mu1 + pi);
    ank_L = prepro + (presup - prepro)*(mu + 2*pi - K_mu1)/(K_mu0 - K_mu1 + pi);
end

% 6. Complete leg motion
eta_R = eta_home + eta_Rleg_lift;
phi_Rleg = phi_Rleg_home + [hip_R;0;0] + RLegSwing; 
phi_Rfoot = phi_Rfoot_home + [ank_R;0];

eta_L = eta_home + eta_Lleg_lift;
phi_Lleg = phi_Lleg_home + [hip_L;0;0] + LLegSwing; 
phi_Lfoot = phi_Lfoot_home + [ank_L;0];

eta = [eta_R;eta_L];
phi_leg = [phi_Rleg,phi_Lleg];
phi_foot = [phi_Rfoot,phi_Lfoot];
end

