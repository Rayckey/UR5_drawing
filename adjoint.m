function adj = adjoint(g)
if size(g) ~= [4,4]
    error('Size of g not correct')
end
R = g(1:3,1:3);
p = g(1:3,4);
adj = [R, hat(p)*R
    zeros(3), R];