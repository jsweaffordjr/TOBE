function P = servoangles(ang)

% This function computes the servo motor positions for the BIOLOID robot,
% given the joint angles (in radians) according to the coordinate frame numbering

% C = (300/1023)*[1;1;1;-1;-1;1;-1;1;-1;1;1;-1;1;-1;-1;1;-1;-1];
C = [-1;1;-1;-1;-1;1;-1;-1;1;1;-1;1;1;-1;-1;1;-1;-1];
% B = [150;150;150;-150;-150;195;-105;150;-150;150;150;-150;240;-240;-150;60;-60;-150];
B = [150;150;150;150;150;150;150;150;150;150;150;150;240;240;150;60;60;150];

A = round((1023/300)* (B + ((ang*(180/pi)).*C)));

P = [A(16);A(13);A(17);A(14);A(18);A(15);A(7);A(6);A(8);A(5);A(9);A(4);A(10);A(3);A(11);A(2);A(12);A(1)];


end

