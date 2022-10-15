%--------------------------------------------------------------------------
% Submission of:
% 1.    Fabian Domberg 
% 2.	Rakesh Reddy
% 3.	Tim-Henrik Traving
% 4.	Harsh Yadav
% It is a guidence from the University that we should not provide our name
% and student ID together for the reason of data security
%--------------------------------------------------------------------------

function [ f ] = perceptronOutput( x, w )

% Input
% -----
%
% x ... Data point. (column vector)
%
% w ... Weight vector. (clumn vector)
%       [ w_1; w_2 ]

% Output
% ------
%
% f ... Perceptron output characteristic.

% ---------------------------------
f = w'*x;
% ---------------------------------

end