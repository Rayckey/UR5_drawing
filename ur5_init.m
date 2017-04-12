function handles = ur5_init(vrep, id)
% Initialize ur5 -- based on code written for youbot

% Original (C) info here:
% (C) Copyright Renaud Detry 2013.
% Distributed under the GNU General Public License.
% (See http://www.gnu.org/copyleft/gpl.html)
% 
% All modifications (C) Copyright Noah Cowan 2015

% Retrieve all handles, and streams joints and pose of UR5

handles = struct('id', id);

%% Get Joints Handles
jointNames = {'UR5_joint1','UR5_joint2','UR5_joint3','UR5_joint4',...
    'UR5_joint5','UR5_joint6'};
ur5Joints = -ones(1,6);
for i = 1:6
    [res, ur5Joints(i)] = vrep.simxGetObjectHandle(id, ...
        jointNames{i}, vrep.simx_opmode_oneshot_wait); 
    vrchk(vrep, res);
end
for i = 1:6,
    vrep.simxSetJointTargetVelocity(id, ur5Joints(i),0, ...
        vrep.simx_opmode_oneshot);
    vrchk(vrep, res);
end

handles.ur5Joints = ur5Joints;

%% Get Link Handles
linkNames = {'UR5_link2','UR5_link3','UR5_link4',...
    'UR5_link5','UR5_link6','UR5_link7'};

ur5Links = -ones(1,6);
for i = 1:6
    [res, ur5Links(i)] = vrep.simxGetObjectHandle(id, ...
        linkNames{i}, vrep.simx_opmode_oneshot_wait); 
    vrchk(vrep, res);
end
handles.ur5Links = ur5Links;

%% Other Handles
[res, handles.Frame0] = vrep.simxGetObjectHandle(id, ...
    'Frame0', vrep.simx_opmode_oneshot_wait);
vrchk(vrep, res);

[res, handles.Sphere] = vrep.simxGetObjectHandle(id, ...
    'Sphere', vrep.simx_opmode_oneshot_wait);
vrchk(vrep, res);

[res, handles.Plane] = vrep.simxGetObjectHandle(id, ...
    'Plane', vrep.simx_opmode_oneshot_wait);
vrchk(vrep, res);

[res, ur5Ref] = vrep.simxGetObjectHandle(id, 'UR5', ...
    vrep.simx_opmode_oneshot_wait); 
vrchk(vrep, res);

[res, ur5Gripper] = vrep.simxGetObjectHandle(id, 'UR5_connection', ...
    vrep.simx_opmode_oneshot_wait);
vrchk(vrep, res);


handles.ur5Ref = ur5Ref;
handles.ur5Gripper = ur5Gripper;

res = vrep.simxGetObjectPosition(id, ur5Ref, -1,...
    vrep.simx_opmode_streaming); 
vrchk(vrep, res, true);
res = vrep.simxGetObjectOrientation(id, ur5Ref, -1,...
    vrep.simx_opmode_streaming); 
vrchk(vrep, res, true);

res = vrep.simxGetObjectPosition(id, ur5Gripper, ur5Ref, vrep.simx_opmode_streaming); vrchk(vrep, res, true);
res = vrep.simxGetObjectOrientation(id, ur5Gripper, ur5Ref, vrep.simx_opmode_streaming); vrchk(vrep, res, true);

for i = 1:6,
  res = vrep.simxGetJointPosition(id, ur5Joints(i),...
      vrep.simx_opmode_streaming);
  vrchk(vrep, res, true);
end
for i = 1:6
    res = vrep.simxGetObjectPosition(id, ur5Joints(i),...
        ur5Ref,vrep.simx_opmode_streaming);
    vrchk(vrep,res,true);
end

vrep.simxGetPingTime(id);

end
