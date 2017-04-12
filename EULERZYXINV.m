function Angles= EULERZYXINV(input)

[rows,coloumns] = size(input);

if rows ~= 3 || coloumns ~= 3
    error('The Size Is Not Right ')
end

    a = atan2(-input(2,3),input(3,3));
    c2 = sqrt(input(2,3)^2 +input(3,3)^2 );
    b = atan2(input(1,3), c2);
    c = atan2(-input(1,2),input(1,1));
    Angles = [a;b;c];

end