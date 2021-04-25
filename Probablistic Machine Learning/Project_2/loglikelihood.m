function [cost, grad] = loglikelihood(trainX,trainY,params)

sigma = params(1);
l = params(2);
K = RBF_kernel(trainX,trainX,sigma,l);
n = length(trainX);
K_inv = inv(K);

cost = 0.5*(trainY'/K)*trainY + 0.5*log(norm(K)) + (n/2)*log(2*pi);

alpha = K\trainY;
d = 0.001;
sigma_new = sigma + d;
l_new = l + d;
K_new_sigma = RBF_kernel(trainX,trainX,sigma_new,l);
grad = zeros(length(params),1);
grad(1) = 0.5*trace( (alpha*alpha'-K_inv)*((K_new_sigma-K)./d) );
K_new_l = RBF_kernel(trainX,trainX,sigma,l_new);
grad(2) = 0.5*trace( (alpha*alpha'-K_inv)*((K_new_l-K)./d) );
end
