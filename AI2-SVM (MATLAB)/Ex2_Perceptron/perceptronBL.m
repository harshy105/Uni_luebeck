%--------------------------------------------------------------------------
% Submission of:
% 1.    Fabian Domberg 
% 2.	Rakesh Reddy
% 3.	Tim-Henrik Traving
% 4.	Harsh Yadav
% It is a guidence from the University that we should not provide our name
% and student ID together for the reason of data security
%--------------------------------------------------------------------------

function [ w, iter, exitflag ] = perceptronPBPL( X, w, eta, maxIter )

% Input
% -----
%
% X        ... Data points and class labels.
%              [ x_11, x_12, s_1;
%                x_21, x_22, s_2;
%                x_31, x_32, s_3;
%                ...              ]
[m,n] = size(X);
% extracting feature from X matrix and adding one to each data set
% it will reduce the equation from w'*x > S to w'*x > 0
% x = [ones(m,1) X(:,1:n-1)];
x = X(:,1:n-1);

% extracting the classifiers from the X matrix
s = X(:,n);

%changing the sign of the netgaive data set
x = [x(s==1,:);-x(s==-1,:)];

%normalising the data set, not required for this exercise since all the
%data set are already normalised, still it is a good exercise to implement
%it

% for index=1:m;    
%     p = norm(x(index,:));
%     for j = 1:n
%         x(index,j) = x(index,j)/p;
%     end
% end
%
% w        ... Initial weight vector.
%              [ w_1; w_2 ]
% add the weight corresponding to the feature with constant value 1
% w = [rand; w];
%
% eta      ... Step size.
%
% maxIter  ... Maximum number of iterations.

% Output
% ------
%
% w        ... Final weight vector.
%
% iter     ... Number of iterations needed.
%
% exitflag ... Exit flag.
%              0 = No solution found, maximum number of iterations reached.
%              1 = Solution found.

% Determine number of data points.
% n = size(X, 1);

% Initialize iteration counter, exit condition and exit flag.
iter     = 0;
exit     = 0;
exitflag = 0;

% While exit condition not met... 
while ((~exit) && (iter < maxIter))
    % Increment iteration counter.
    iter = iter + 1;    
    % ---------------------------------
    
    %looping over all the data set to check if any of the perceptron output is zero
    
    index = 1; %initialsing the data index in every iteration to check the perceptron value 
    exit = 1; % initilzing exit condition to be true i.e. all the perceptron give positive output 
    
    %execute loop till the first perceptron gives negative output
    while ((exit) && (index <= m))
        percep_out = perceptronOutput(x(index,:)',w); %checking the perceptron value
        if (percep_out < 0) 
            w = w + eta*x(index,:)'; % updating the weight vector
            exit = 0; %exit condition is not met hence the iteration continous
        else
            index = index+1; %update index
        end
    end
    
    % ---------------------------------
    
end

%exclude the weight corresponding the constant feature 1
w = w(2:n);

%set existfalg to Exit Flag
exitflag = exit;

end