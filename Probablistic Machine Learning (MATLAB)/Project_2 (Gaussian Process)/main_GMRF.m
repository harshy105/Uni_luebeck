clear all;
close all;
clc;

% Load data
load('TempFieldDataSubset.mat')

% Randomize
random_permutation = randperm(size(trainX,1));
trainX = trainX(random_permutation,:);
trainY = trainY(random_permutation);

%noise
var_noise = 0.1; % sigma_n^2

% RBF parameters
%% GMRF 
sigma = 1; 
l = 0.15;
pred_testY = zeros(length(testY),1);

for i = 1:length(testY)

    m = 8; %nearest 10 random points for spatial field
    idx = knnsearch(trainX,testX(i,:),'K',m);
    field = trainX(idx,:);

    Q = 10^6*eye(length(field)); %Hyperparameter 
    big_lambda = RBF_kernel(trainX,field,sigma,l);
    small_lambda = RBF_kernel(field,testX(i,:),sigma,l);
    mu = mean(trainY);
    mu_s = mean(trainY(idx));

    Q_bar = Q + var_noise^-2*(big_lambda'*big_lambda);
    y_bar = var_noise^-2*big_lambda'*(trainY-mu);

    mu_predict = mu_s + small_lambda'*(Q_bar\y_bar);
    sigma_predict = small_lambda'*(Q_bar\small_lambda);

    pred_testY(i) = mu_predict;
end

error= norm(pred_testY-testY);
fprintf('MSE with GMRF is %d\n', error);

Plot_prediction(testX, testY, pred_testY);
title('Temperature prediction using GMRF')


