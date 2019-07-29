function [ C ] = rotValidate( C )
% VALIDATEROTATION causes an error if the rotation matrix is not orthonormal
%  
% From: Timothy D Barfoot and Paul T Furgale, 
%       Associating Uncertainty with Three-Dimensional Poses for use in Estimation Problems
%		DOI: 10.1109/TRO.2014.2298059
%       tim.barfoot@utoronto.ca, paul.furgale@mavt.ethz.ch
%
% input:
%   C: a 3x3 matrix
%
% output:
%   C: If C is a valid rotation matrix, it is returned.
%      Otherwise, an error is thrown.
%

validateattributes(C,{'double'},{'size',[3,3]});

CtC = C'*C;

E = CtC - eye(3);
err = max(max(abs(E)));

if err > 1e-10
    error('The rotation matrix is not valid. Maximum error: %f10.15\n', err);
end

end
