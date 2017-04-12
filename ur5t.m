function T = ur5t(theta,i)
%Input of usefal constants
d =[0.089159,0,0,0.10915,0.09465,0.0823];
a =[0,-0.425,-0.39225,0,0,0];
alpha = [pi/2, 0,0,pi/2,-pi/2,0];
% Calucalation for parameters
T = [ cos(theta), -sin(theta)*cos(alpha(i)) , sin(theta)*sin(alpha(i)),a(i)*cos(theta);
sin(theta),cos(theta)*cos(alpha(i)),-cos(theta)*sin(alpha(i)), a(i)*sin(theta);
0,sin(alpha(i)),cos(alpha(i)),d(i);
0,0,0,1];

end
