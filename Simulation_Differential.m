clc; clear all; close all;
%Differential Kinematics

%Entering the name of the figure
NAME = 'drake.PNG';
%Using the function imageoutlining to retrive and display oulines
[DaLines,nol,BWpic] = imageoutlining(NAME);

% Input of the points on the white board
EdgesP = [83.8,271.69 -83.80;-274.25 -266.67 -268.19; 978.07 896.86 ...
    584.67]/1000;
% Use boarddetect to determine the board normal
[n,center,Ori] = boarddetect(EdgesP);
% Since the input points do not met specification, maually enter center
% point
center = [-83.80;-268.19;584.67]/1000;

% Adjust for the difference in calcaltion and Vrep simulation
frametwist = [hat([0,-1,0]),[0;0;0];
    0 0 0 0];
goffset = expm(frametwist*pi/2);
%Assemble the plane transformation
gp = [Ori , center
    zeros(1,3) ,1];

%% Start Vrep and Preperation
disp('Program started');
%launch remoteApi
vrep=remApi('remoteApi');

vrep.simxFinish(-1);
id = vrep.simxStart('127.0.0.1', 19997, true, true, 2000, 5);
res = vrep.simxStartSimulation(id, vrep.simx_opmode_oneshot_wait);
if id < 0,
    disp('Failed connecting to remote API server. Exiting.');
    vrep.delete();
    return;
end
fprintf('Connection %d to remote API server open.\n', id);

% Make sure we close the connexion whenever the script is interrupted.
cleanupObj = onCleanup(@() cleanup_vrep(vrep, id));

% Retrieve all handles, links and stream arm and wheel joints
% see ur5_init.m for details
h = ur5_init(vrep, id);

%set constrains
timestep = .2;

%move the board to te desired location
movef(id,vrep,gp,h.Plane);

% Set the arm to its starting configuration:
startingJoints = zeros(1,6);
for i = 1:6,
    res = vrep.simxSetJointTargetPosition(id, h.ur5Joints(i),...
        startingJoints(i),...
        vrep.simx_opmode_oneshot);
    vrchk(vrep, res, true);
end
pause(2);
% More significant figures for more actuate position
format long
%% Robot Manipulation

% Retrive the current position of all the joints
for i = 1:6,
    [res, position(:,i)] = vrep.simxGetObjectPosition(id, h.ur5Joints(i),...
        -1, vrep.simx_opmode_oneshot_wait);
    vrchk(vrep, res);
end

toolFrame = {'handles.FrameT1','handles.FrameT2','handles.FrameT3'};
%This loop creates 6 additional frames, and attach them to each link

%This loop manipulates the robot
z = 0;
pkkkk = 1;
for k = 1:nol;
    %import the outline
    OutLine = DaLines{k};
    [rows,cols] = size(OutLine);
    for i = 1:5:rows;
        %convert points to points on a 15x15 board
        [xy] = OutLine(i,:)/length(BWpic)*0.15;
        xy = xy -0.075;
        %convert the points to board
        pointp =gp* [xy';z;1];
        g = [gp(1:4,1:3),pointp]*goffset;
        
        toolFrames{k} = copyf(id,vrep,g,h.Sphere);
        %Inverse Kinematics for solution
        Q = ur5inv(g);
        s =1;
        solution =  Q(:,s);
        
        if i == 1
            for n = 1:6;
                %Setting the UR5 to move to the Target positons
                vrep.simxSetJointTargetPosition(id, h.ur5Joints(n),...
                    solution(n), ...
                    vrep.simx_opmode_oneshot);
            end
            for p = 1:6;
                % This loop ensures that the robot reaches its postion before the
                % next step
                err=1;  %set initial errr as 1 to initiate the while loop
                while err > 0.02
                    %pretty small torlerance
                    pause(0.1);
                    %this for loop checks for actual joint position
                    [res, activeJointPosition(p)] =...
                        vrep.simxGetJointPosition(id, h.ur5Joints(p),...
                        vrep.simx_opmode_buffer);
                    vrchk(vrep, res);
                    %computation of error
                    err = abs( activeJointPosition(p) - solution(p));
                    if abs(err- 2*pi) <0.02;
                        err = 0;
                    end
                    
                end
            end
        else
            %If not the first point, retrive the joint angles
            for q = 1:6;
                [res, activeJointPosition(q)] =...
                    vrep.simxGetJointPosition(id, h.ur5Joints(q),...
                    vrep.simx_opmode_buffer);
                vrchk(vrep, res);
            end
            qk = activeJointPosition';
            %calculate the spatial jacobian
            [Jacob,er] = bjacob(qk);
            %Use of differential kinematics equation
            gtrue = forwardK(qk);
            dg  = inv(gtrue)*g - eye(4);
            solution = qk + inv(Jacob)*unhat(dg);
            error = solution- qk;
            
            for n = 1:6;
                %Setting the UR5 to move to the Target positons
                vrep.simxSetJointTargetPosition(id, h.ur5Joints(n),...
                    solution(n), ...
                    vrep.simx_opmode_oneshot);
            end
        end
        
    end
    
    %At the end of the the line, return to home position
    for i = 1:6;
        res = vrep.simxSetJointTargetPosition(id, h.ur5Joints(i),...
            startingJoints(i),...
            vrep.simx_opmode_oneshot);
        vrchk(vrep, res, true);
    end
end
