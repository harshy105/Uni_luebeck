function [alphas, idx, d] = smoTrain( X, y, K, C, eps, maxIter )

% Input
% -----
%
% X        ... Data points.
%              [ x_11, x_12;
%                x_21, x_22;
%                x_31, x_32;
%                ...              ]
%
% y        ... Class labels.
%              [ s_1; s_2; s_3; ... ]
%
% K        ... Kernel.
%              @(x, y) ...
%
% C        ... SVM hyperparameter.
%
% eps      ... User-defined epsilon for KKT conditions
%
% maxIter  ... Maximum number of iterations.

% Output
% ------
%
% alphas   ... Lagrange multipliers.
%
% idx      ... Indices of non-zero alphas.
%
% d        ... Parameter d.

% 1.    Fabian Domberg 
% 2.	Rakesh Reddy
% 3.	Tim-Henrik Traving
% 4.	Harsh Yadav

% YOUR IMPLEMENTATION GOES HERE...

m = length(y);
alphas = zeros(m,1);
d = 0;
num_iter = 0;
kkt = 0; 
%maxIter = 100; %the algorithm also works fine for 100 iterations. 

while (num_iter < maxIter) && (kkt == 0)
    num_iter = num_iter + 1;
    kkt = 1;
    for i = 1:m       
        %calculation of f_i
        f_i = d;
        for l = 1:m
            f_i = f_i + alphas(l)*y(l)*K(X(i,:),X(l,:));
        end
  
        % check if the KKT is satified
        if ((y(i)*f_i - 1)<-eps && alphas(i)<C) || ((y(i)*f_i - 1)>eps && alphas(i)>0)
            kkt = 0;
            % compute j and store alpha_j
            rand = randi(m-1,1);
            jlist = [1:i-1,i+1:m];
            j = jlist(rand);
            alphas_oldj = alphas(j);
            
            %compute eta
            eta = -K(X(i,:),X(i,:)) - K(X(j,:),X(j,:)) + 2*K(X(i,:),X(j,:));
            %skip if eta >=0
            if eta >= 0
                continue
            end
                
            %compute L and H
            if y(i) == y(j)
                L = max(0, alphas(i) + alphas(j) - C);
                H = min(C, alphas(i) + alphas(j));
            else
                L = max(0, alphas(j) - alphas(i));
                H = min(C, C + alphas(j) - alphas(i));                    
            end            
            % skip if L = H
            if L == H
                continue
            end
            
            %compute Ei and Ej
            f_j = d;
            for l = 1:m
                f_j = f_j + alphas(l)*y(l)*K(X(j,:),X(l,:));
            end
            E_i = f_i - y(i);
            E_j = f_j - y(j);

            %calculate alpha_j
            alpha_newj = alphas(j) - y(j)*(E_i-E_j)/eta;

            % clip alpha_j
            if alpha_newj > H
                alphas(j) = H;
            elseif alpha_newj < L
                alphas(j) = L;
            else
                alphas(j) = alpha_newj;
            end
            
            % skip if the change in alpha_j is very small
            if abs(alphas(j)-alphas_oldj)<exp(-6)
                continue
            end

            % calculate alpha_i
            alphas(i) = alphas(i) + y(i)*y(j)*(alphas_oldj-alphas(j));
            
            % calculate d from the KKT ccondition on the i
            f_inotd = 0;
            for l = 1:m
                f_inotd = f_inotd + alphas(l)*y(l)*K(X(i,:),X(l,:));
            end   
            d = y(i)-f_inotd ;                                              
        end        
    end
end

idx = abs(alphas) > exp(-3); % compute the ids of the non zero alphas

end

