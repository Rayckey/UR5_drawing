function RMZYX= EULERZYX(input)

[rows,coloumns] = size(input);

if rows ~= 3 || coloumns ~= 1
    error('The Size Is Not Right ')
end

[RMX] = ROTX(input(1));
[RMY] = ROTY(input(2));
[RMZ] = ROTZ(input(3));
RMZYX = RMX*RMY*RMZ;

end