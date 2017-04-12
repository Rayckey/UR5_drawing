%% Dummy Main Function

% Must have these two lines:
Socket_conn = 1;  
REAL = 0;               % set to 1 to test on real robot

if ~REAL
    % initialize vrep API  
    
    % get handles for frames and joints
    
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

theta = [th1 th2 th3 th4 th5 th6];

% Move robot to a given theta
move_robt(id,vrep,h,theta,Socket_conn,REAL);

% Get pose
get_pose(id,vrep.h,Socket_conn,REAL);



