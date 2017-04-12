function movef(id,vrep,g,h,hrel)

% Error Check
[rows,coloumns] = size(g);
if rows ~= 4 || coloumns ~= 4
    error('The Size of Transformation Matrix Is Not Correct');
end
if ~exist('hrel','var')
    hrel = -1;      % Set relative frame to -1 if not defined
end
%Extract rotation matrix and translation vector
Rotation_M = g(1:3,1:3);
v = g(1:3,4);
%convertion to Euler Angles
angles = EULERZYXINV(Rotation_M);
% set Orientation and Position
[res] = vrep.simxSetObjectOrientation(id, h,...
    hrel,angles, ...
    vrep.simx_opmode_oneshot);
[res] =vrep.simxSetObjectPosition(id, h,...
    hrel,v', ...
    vrep.simx_opmode_oneshot);
