A = @(alpha) [0 0 0;0 alpha 1; 0 0 1];
alpha = fzero(@A); 