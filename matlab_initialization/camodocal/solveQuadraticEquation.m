% solve an equation ax^2+bx+c
% [b_equ, x1, x2]: 
%   b_equ: if solvable equation
%   x1, x2: solved value
function [b_equ, x1, x2] = solveQuadraticEquation(a, b, c) 
    if (abs(a) < 1e-12)
        x1 = -c / b;
        x2 = -c / b;
        b_equ = true;
    end
    delta2 = b*b - 4.0*a*c;

    if (delta2 < 0.0)
        b_equ = false;
    end
    delta = sqrt(delta2);

    x1 = (-b + delta) / (2.0 * a);
    x2 = (-b - delta) / (2.0 * a);
    b_equ = true;
end