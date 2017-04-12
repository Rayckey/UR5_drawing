clc; clear all; close all;

%% Dummy Main Function

% Must have these two lines:
Socket_conn = 1;
REAL = 1;               % set to 1 to test on real robot

if ~REAL
    % initialize vrep API
    
    % get handles for frames and joints
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
    
    pause(2)
    
else
    Robot_IP = '172.22.22.2';
    Socket_conn = tcpip(Robot_IP,30000,'NetworkRole','server');
    fclose(Socket_conn);
    disp('Press play on robot')
    fopen(Socket_conn);
    disp('Connected');
    id = 1;
    vrep = 0;
    h=0;
end

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
center = [-183.80;-268.19;584.67]/1000;

% Adjust for the difference in calcaltion and real robot
frametwist = [hat([0,-1,0]),[0;0;0];
    0 0 0 0];

goffset = expm(frametwist*pi);
%Assemble the plane transformation

gp = [Ori , center
    zeros(1,3) ,1]

%set constrains
timestep = .2;

% More significant figures for more actuate position
format long

%% Robot Manipulation

toolFrame = {'handles.FrameT1','handles.FrameT2','handles.FrameT3'};

%This loop manipulates the robot
z = 0;
for k = 1:nol;
    %import the outline
    OutLine = DaLines{k};
    [rows,cols] = size(OutLine);
    for i = 1:5:rows;
        %convert points to points on a 15x15 board
        [xy] = OutLine(i,:)/length(BWpic)*0.15;
        xy = xy -0.075;
        pointp =gp* [xy';z;1];
        %convert the points to board
        g = [gp(1:4,1:3),pointp] *goffset;
        %Inverse Kinematics for solution
        Q = ur5inv(g);
        %Pick only one solution
        s =1;
        solution =  Q(:,s)
        theta = solution';
        % Move robot to a given theta
        if i == 1
            %If it is the first point, lift pen for 2 cm
            Q = ur5inv(g+[zeros(3) [0, 0.02,0]'
                0 0 0 0]);
            theta = Q(:,1);
            move_robt(id,vrep,h,theta,Socket_conn,REAL);
            pause(1);
        else
            %move the robot to the desired location
            move_robt(id,vrep,h,theta,Socket_conn,REAL);
        end
    end
    %at the end of the line, lift pen
    Q = ur5inv(g+[zeros(3) [0, 0.02,0]'
        0 0 0 0]);
    theta = Q(:,1);
    move_robt(id,vrep,h,theta,Socket_conn,REAL);
    pause(1)
end
