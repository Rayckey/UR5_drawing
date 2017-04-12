function [twist] = unhat(hatty)
if size(hatty) ~= [4,4]
    error('Size is not rigt')
end

w(1) = - hatty(2, 3);
w(2) = hatty(1,3);
w(3) = -  hatty(1,2);
w = zeros(1,3);
v = hatty(1:3,4);

twist = [v;w'];