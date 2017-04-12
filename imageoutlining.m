function [DaLines,nol,BWpic] = imageoutlining(NAME)
% Read the image 
Dapic= imread(NAME);

figure
imshow(Dapic)
% convert the black and write
BWpic = im2bw(Dapic,graythresh(Dapic));
figure
BWpic = flipud(BWpic);
imshow(BWpic)
% Start detecting the outine
DaLines = bwboundaries(BWpic);
figure
hold on
%save the outline then show it on figure.
for nol = 1:length(DaLines);
    OutLine = DaLines{nol};
    plot(OutLine(:,2),OutLine(:,1),'b','LineWidth',2);
end
hold off
end
