function [Jacob,error] = bjacob(angles)
% Warning
% This body jacobian only works on UR5... or not

%Manual input of gst(0), where the tool frame will be when all angles
%are zeros
g0 = [eye(3) [-0.1850; 0; 1];
    0 0 0 1];

% Collection of all w
w = [0,-1,-1,-1,0,-1;
    0,0,0,0,0,0;
    1,0,0,0,1,0];
position = [6.1084812e-08,-0.070308745,-0.070350021,-0.070386007,...
    -0.11009524,-0.095675573;...
    -1.6296529e-08,-2.6921043e-08,-1.2288365e-07,...
    -2.4015026e-07,-2.9549119e-07,-3.0252340e-07;...
    0.023149960,0.089194328,0.51429677,0.90644622,0.95201683,1.0011935];

for i =1:6;
    %Create the product of expoenantial for each joint
    %for each of the joint, V component of the twist is constructed
    v(:,i) = cross(-w(:,i),position(:,i));
    %The twist is constructed, this value is not used
    twist{i} = [ v(:,i); w(:,i)];
    %The twist hat is constructed
    twist_hat{i} = [hat(w(:,i)),v(:,i);
        0 0 0 0];
    
    %Overwrite the coresponding g with the twist exponential
    g{i} = expm(twist_hat{i}*angles(i));
end
%error check, in case the forward kinematics does not match
error = g{1}*g{2}*g{3}*g{4}*g{5}*g{6}*g0;
% Body jacobian calculation
for i = 1:6;
    Adp = eye(4);
    for j = i:6;
        Adp = Adp*g{j};
    end
    Adp = Adp*g0;
    adj = adjoint(Adp);
    twistdagger(:,i) = inv(adj)*twist{i};
end
% Assemble the Body Jacobian 
Jacob = twistdagger;
end
