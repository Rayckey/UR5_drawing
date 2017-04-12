function Q = ur5inv(gd)
% WARNING this code only works on UR5
% Applying this code on any other robot may result in
% face slapping rogue robots
d =[0.089159,0,0,0.10915,0.09465,0.0823];
a =[0,-0.425,-0.39225,0,0,0];
alpha = [pi/2, 0,0,pi/2,-pi/2,0];

%% One twist for the frame orientation offset
frametwist = [hat([0,-1,0]),[0;0;0];
    0 0 0 0];
gd = gd*expm(frametwist*pi/2);
gd = gd + [0 0 0 -0.0001
    0 0 0 0.0072
    0 0 0 -0.0004
    0 0 0 0];

%% Lets do this Ryan Keating
% Solving for theta 1
p05 = gd*[0;0;-d(6);1] - [0;0;0;1];
theta1(1) = atan2(p05(2),p05(1))+acos(d(4)/sqrt(p05(1)^2 + p05(2)^2))+pi/2;
theta1(2) = atan2(p05(2),p05(1))-acos(d(4)/sqrt(p05(1)^2 + p05(2)^2))+pi/2;
%solving for theta 5
for i = 1:2;
    p16z{i} = gd(1,4)*sin(theta1(i)) - gd(2,4)*cos(theta1(i));
    theta5(i) = acos((p16z{i}-d(4))/d(6));
    theta5(i+2) = -acos((p16z{i}-d(4))/d(6));
    
    %solving for theta 6
    t01{i} = ur5t(theta1(i),1);
    t16{i} = inv(t01{i})*gd;
    tempg61 = inv(t16{i});
    if sin(theta5(i)) == 0  || sin(theta5(i+2)) == 0
        theta6(i) = 0;
        theta6(i+2) = 0;
    else
        theta6(i) = atan2(-tempg61(2,3)/sin(theta5(i)),tempg61(1,3)/sin(theta5(i)));
        theta6(i+2) = atan2(-tempg61(2,3)/sin(theta5(i+2)),tempg61(1,3)/sin(theta5(i+2)));
    end
end

%solution reorganize
theta1 =[theta1,theta1];
t16 = [t16,t16];

for j = 1:4;
    %solving for theta 3
    t14{j} = t16{j}*inv(ur5t(theta5(j),5)*ur5t(theta6(j),6));
    p13 = t14{j}*[0;-d(4);0;1] - [0;0;0;1];
    
    theta3(j) = acos( ((norm(p13))^2 -a(2)^2 -a(3)^2)/(2*a(2)*a(3)));
    theta3(j+4) =-acos( ((norm(p13))^2 -a(2)^2 -a(3)^2)/(2*a(2)*a(3)));
    if isreal(theta3(j))==0 || isreal(theta3(j+4))==0
        error('Warning: theta 3 is unsolvable')
    end
    %solving for theta 2;
    theta2(j) = -atan2(p13(2),-p13(1)) + asin(a(3)*sin(theta3(j))/...
        norm(p13));
    theta2(j+4) = -atan2(p13(2),-p13(1)) + asin(a(3)*sin(theta3(j+4))/...
        norm(p13));
end
%Reorganizing data
theta1 = [theta1,theta1];
theta5 = [theta5,theta5];
theta6 = [theta6,theta6];
t14 = [t14,t14];
for k = 1:8;
    %finally solving for theta 4;
    t34 = inv(ur5t(theta2(k),2)*ur5t(theta3(k),3))*t14{k};
    
    theta4(k) = atan2(t34(2,1),t34(1,1));
end
Q = [theta1;theta2;theta3;theta4;theta5;theta6] + ...
    repmat([pi/2;pi/2;0;pi/2;0;pi/2],1,8);
