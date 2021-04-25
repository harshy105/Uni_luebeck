clear all;
close all;
clc;

%% Load data
load('TempFieldDataSubset.mat')

% Randomize
random_permutation = randperm(size(trainX,1));
trainX = trainX(random_permutation,:);
trainY = trainY(random_permutation);

%noise
var_noise = 0.1; % sigma_n^2

%% GP with RBF kernel
sigma = 1; 
l = 0.15;
K = RBF_kernel(trainX, trainX, sigma, l) + var_noise*eye(length(trainX));
K_s = RBF_kernel(trainX, testX, sigma, l);
K_ss = RBF_kernel(testX, testX, sigma, l);
mu = zeros(length(trainX),1);
mu_s = zeros(length(testX),1);

mu_predict = mu_s + (K_s'/K)*(trainY-mu);
sigma_predict = K_ss - (K_s'/K)*K_s;

pred_testY = mu_predict;
error = norm(pred_testY-testY);
fprintf('Error with RBF Kernel and mean(.) = 0 is %d\n', error);

Plot_prediction(testX, testY, pred_testY);
title('RBF (isotropic) Kernel and mean(.) = 0')

%% Different mean function
% periodic Kernel
sigma = 1; 
l = 0.15;
p = 10;
K = periodic_kernel(trainX, trainX, sigma, l, p) + var_noise*eye(length(trainX));
K_s = periodic_kernel(trainX, testX, sigma, l, p);
K_ss = periodic_kernel(testX, testX, sigma, l, p);
mu = zeros(length(trainX),1);
mu_s = zeros(length(testX),1);

mu_predict = mu_s + (K_s'/K)*(trainY-mu);
sigma_predict = K_ss - (K_s'/K)*K_s;

pred_testY = mu_predict;
error = norm(pred_testY-testY);
fprintf('Error with Perodic Kernel and mean(.) = 0 is %d\n', error);

Plot_prediction(testX, testY, pred_testY);
title('Periodic(isotropic) Kernel and mean(.) = 0')

 % Linear Kernel
sigma = 0.1; 
sigma_b = 1;
c = [(max(trainX(:,1))+min(trainX(:,1)))/2, (max(trainX(:,2))+min(trainX(:,2)))/2];
K = linear_kernel(trainX, trainX, sigma, sigma_b, c) + var_noise*eye(length(trainX));
K_s = linear_kernel(trainX, testX, sigma, sigma_b, c);
K_ss = linear_kernel(testX, testX, sigma, sigma_b, c);
mu = zeros(length(trainX),1);   
mu_s = zeros(length(testX),1);

mu_predict = mu_s + (K_s'/K)*(trainY-mu);
sigma_predict = K_ss - (K_s'/K)*K_s;

pred_testY = mu_predict;
error = norm(pred_testY-testY);
fprintf('Error with Linear Kernel and mean(.) = 0 is %d\n', error);

Plot_prediction(testX, testY, pred_testY);
title('Linear(dot product) Kernel and mean(.) = 0')

%% Different mean functions

% Mean value of y as the prior mean of y

sigma = 1; 
l = 0.15;
K = RBF_kernel(trainX, trainX, sigma, l) + var_noise*eye(length(trainX));
K_s = RBF_kernel(trainX, testX, sigma, l);
K_ss = RBF_kernel(testX, testX, sigma, l);
mu = mean(trainY).*ones(length(trainX),1);
mu_s = mean(trainY).*ones(length(testX),1);

mu_predict = mu_s + (K_s'/K)*(trainY-mu);
sigma_predict = K_ss - (K_s'/K)*K_s;

pred_testY = mu_predict;
error= norm(pred_testY-testY);
fprintf('Error with RBF Kernel and mean(.) = mean(y) is %d\n', error);

Plot_prediction(testX, testY, pred_testY);
title('RBF Kernel and \mu = mean(y)')

% Linear regression to claculate the prior mean of y 
trainX_mu = mean(trainX,1);
testX_mu = mean(testX,1);
testX_std = std(testX,0,1);
trainX_std = std(trainX,0,1);
trainX_norm(:,1) = (trainX(:,1) - trainX_mu(1)) ./trainX_std(1);
trainX_norm(:,2) = (trainX(:,2) - trainX_mu(2)) ./trainX_std(2);
testX_norm(:,1) = (testX(:,1) - testX_mu(1)) ./testX_std(1);
testX_norm(:,2) = (testX(:,2) - testX_mu(2)) ./testX_std(2);
testX_offset = [ones(length(testX),1), testX_norm];
trainX_offset = [ones(length(trainX),1), trainX_norm];
lambda = 1;
A = trainX_offset;
w = (A'*A + lambda.*eye(size(A',1)))\(A'*trainY);
mu = (trainX_offset*w);
mu_s = (testX_offset*w);

mu_predict = mu_s + (K_s'/K)*(trainY-mu);
sigma_predict = K_ss - (K_s'/K)*K_s;

pred_testY = mu_predict;
error = norm(pred_testY-testY);
fprintf('Error with RBF Kernel and mean(.) from Linear Regression is %d\n', error);

Plot_prediction(testX, testY, pred_testY);
title('RBF Kernel and \mu from Linear Regression')


%% Meta learning Log-Likelihood 
parameters_initial = [1,0.15];
cost_function = @(params) loglikelihood(trainX,trainY,params);
options = optimset('Display','iter', 'GradObj', 'on','MaxIter',30);
lb = [0;0];
ub = [1;100];
% [parameters_opt,cost_opt] = fmincon(cost_function,parameters_initial,[],[],[],[],lb,ub,[],options);
[parameters_opt,cost_opt,exitflag] = fminunc(cost_function,parameters_initial,options);

sigma = parameters_opt(1);
l = parameters_opt(2);
K = RBF_kernel(trainX, trainX, sigma, l) + var_noise*eye(length(trainX));
K_s = RBF_kernel(trainX, testX, sigma, l);
K_ss = RBF_kernel(testX, testX, sigma, l);
mu = zeros(length(trainX),1);   
mu_s = zeros(length(testX),1);

mu_predict = mu_s + (K_s'/K)*(trainY-mu);
sigma_predict = K_ss - (K_s'/K)*K_s;

pred_testY = mu_predict;
error = norm(pred_testY-testY);
fprintf('Error with RBF Kernel with Meta Learning is %d\n', error);

Plot_prediction(testX, testY, pred_testY);
title('Log-Likelihood optimised RBF Kernal and mean(.) = 0')




