function hnew = copyf(id,vrep,g,h,hrel)
% Error Check
[rows,coloumns] = size(g);
if rows ~= 4 || coloumns ~= 4
    error('The Size of Transformation Matrix Is Not Correct');
end
if ~exist('hrel','var')
    hrel = -1;      % Set relative frame to -1 if not defined
end
%Copy a new frame
[res, hnew] =vrep.simxCopyPasteObjects(id,h,...
    vrep.simx_opmode_oneshot_wait);
%transfer to movef function
movef(id,vrep,g,hnew,hrel);