function [n,center,Ori] = boarddetect(EdgesP)
%calcualte three vectors
victors(:,1) = EdgesP(:,1) - EdgesP(:,2);
victors(:,2) = EdgesP(:,2) - EdgesP(:,3);
victors(:,3) = EdgesP(:,3) - EdgesP(:,1);
% calcualte the noraml vector
n = cross(victors(:,1), victors(:,2));
n = n / norm(n);
% Construt the transmoration matrix
zz = n;
yy = [1;0;0];
xx = -cross(zz,yy);
Ori = [xx, yy, zz];
Orig = [ Ori , zeros(3,1);
    zeros(1,3), 1 ];

% calculate the center of the board, if the three points are on the left,
% top and right edge
EdgesP = [ EdgesP; 1 1 1];
for i = 1:3;
    TranP(:,i) =   inv(Orig)*EdgesP(:,i);
end
[Topy] = max(TranP(2,:)) -0.075;
[Topx] = max(TranP(1,:)) -0.075;
center = Orig*[Topx;Topy;0;1];
end
