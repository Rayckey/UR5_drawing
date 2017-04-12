function g = getf(id,vrep,h,hrel)
% Set default frame if not indicated
if ~exist('hrel','var')
    hrel = -1;
end
% get the orientation
[res,orientation]= vrep.simxGetObjectOrientation(id,...
    h,hrel,vrep.simx_opmode_oneshot_wait);
% get the position
[res,position]= vrep.simxGetObjectPosition(id,...
    h,hrel,vrep.simx_opmode_oneshot_wait);
%convertion to rotation matrix
Rotation_M = EULERZYX(orientation');
%Assemble the Homogeneous matrix
g = [ Rotation_M,position';
    0,0,0, 1];

