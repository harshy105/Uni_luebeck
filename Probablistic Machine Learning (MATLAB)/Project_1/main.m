clear all;
close all;
clc;

%% visulazing the test data
load('TempFieldDataSubset.mat')
figure();
scatter3(testX(1,:),testX(2,:),testy, '.','r');
hold on;
xlabel('Latitute')
ylabel('Logitude')
zlabel('Temperature')
title('Original Data')
hold off;

%% normalization
trainX_mu = mean(trainX,2);
testX_mu = mean(testX,2);
testX_std = std(testX,0,2);
trainX_std = std(trainX,0,2);
trainX_norm(1,:) = (trainX(1,:) - trainX_mu(1)) ./trainX_std(1);
trainX_norm(2,:) = (trainX(2,:) - trainX_mu(2)) ./trainX_std(2);
testX_norm(1,:) = (testX(1,:) - testX_mu(1)) ./testX_std(1);
testX_norm(2,:) = (testX(2,:) - testX_mu(2)) ./testX_std(2);

% Introducing offset
testX_offset = [ones(size(testX,2),1)'; testX_norm];
trainX_offset = [ones(size(trainX,2),1)'; trainX_norm];

%% Ridge Regression
lambda = 1;
A = trainX_offset';
y = trainy;
w = (A'*A + lambda*eye(size(A',1)))\(A'*y);

%Predication of temperature
pred_testy = (testX_offset'*w);

error = norm(pred_testy-testy);
% Visulation of predicted temperature
figure();
scatter3(testX(1,:),testX(2,:),testy, '.','r');
hold on;
scatter3(testX(1,:),testX(2,:),pred_testy,'.','b');
xlabel('Latitute');
ylabel('Logitude');
zlabel('Temperature');
title('Temperature Reconstruction with Ridge Linear Regression')
hold off;

%% Training and Test errors with increasing training data size
train_error = zeros(size(trainX,2),1);
test_error = zeros(size(trainX,2),1);

for i = 1:size(trainX_offset,2)
    A = trainX_offset(:,1:i)';
    y = trainy(1:i);
    w = (A'*A + lambda*eye(size(A',1)))\(A'*y);
    pred_testy = (testX_offset'*w); %prediction of the test values
    test_error(i) = mean((pred_testy-testy).^2); %MSE of test sample
    pred_trainy = (A*w);%prediction of the training values
    train_error(i) = mean((pred_trainy-y).^2); %MSE of the train sample
end

%ploting the errors
figure();
truncate = 500; % for training size < 500 the error is too large, to have a better look those has been excluded
plot((truncate:length(test_error)),test_error(truncate:length(test_error)), 'b');
hold on;
plot((truncate:length(test_error)),train_error(truncate:length(train_error)),'r');
xlabel('Number of Training Points');
ylabel('Error');
hold off;
legend('Test error','Training error');
title(' Error from Ridge Regression Model');

%% Multivariate regression

train_error = [];
test_error = [];
basis_num = [2 5 10 15 20]; % choosing the number of basis
for basis_itr = basis_num
    mu_x1 = linspace(min(trainX_norm(1,:)),max(trainX_norm(1,:)),basis_itr); % x1 coordinates of mu
    mu_x2 = linspace(min(trainX_norm(2,:)),max(trainX_norm(2,:)),basis_itr); % x2 coordinates of mu
    A_train = ones(size(trainX,2),1); % initialising the basis function for training with offset included
    A_test = ones(size(testX,2),1); % initialising the basis function for test with offset included
    cov_inv = inv(0.5.*[mu_x1(2)-mu_x1(1) 0; 0 mu_x2(2)-mu_x2(1)]); % inverse of the covariance

    for i = 1:size(mu_x1,2) 
        for j = 1:size(mu_x2,2)
            mu = [mu_x1(i); mu_x2(j)]; % iterating over all the mu       
            for k = 1:length(trainX)
                basis_train(k,1) = exp(-0.5*(trainX_norm(:,k)-mu)'*cov_inv*(trainX_norm(:,k)-mu));       
            end
            A_train = [A_train basis_train]; % making the basis function for training
            for k = 1:length(testX)
                basis_test(k,1) = exp(-0.5*(testX_norm(:,k)-mu)'*cov_inv*(testX_norm(:,k)-mu));        
            end
            A_test = [A_test basis_test];  % making the basis function for testing
        end
    end

    lambda = 3;
    w = (A_train'*A_train + lambda*eye(size(A_train',1)))\(A_train'*trainy); % calculating weights w

    %Predication and MSE of temperature
    pred_testy = A_test*w;
    pred_trainy = A_train*w;
    
    test_error = [test_error mean((pred_testy-testy).^2)];
    train_error = [train_error mean((pred_trainy-trainy).^2)];
    
    if basis_itr == 2 % Visulation of predicted temperature for 2*2 
        figure();
        scatter3(testX(1,:),testX(2,:),testy, '.','r');
        hold on;
        scatter3(testX(1,:),testX(2,:),pred_testy,'.','b');
        xlabel('Latitute');
        ylabel('Logitude');
        zlabel('Temperature');
        title('Temperature Reconstruction with 2*2 Multivariate Basis function');
        hold off;
        view([0.5,1,0.5]);
    end
end

%plot train and test error with respect to the increasing number of basis
figure();
plot(basis_num, test_error, 'b');
hold on;
plot(basis_num, train_error,'r');
xlabel('Number of Basis');
ylabel('Error');
title('Error with respect to the number of Basis');
hold off;
legend('Test error','Training error');

% Best fit basis 20*20
% Visulation of predicted temperature for 20*20 
figure();
scatter3(testX(1,:),testX(2,:),testy, '.','r');
hold on;
scatter3(testX(1,:),testX(2,:),pred_testy,'.','b');
xlabel('Latitute');
ylabel('Logitude');
zlabel('Temperature');
title('Temperature Reconstruction with 20*20 Multivariate Basis function');
hold off;
view([0.5,1,0.5]);

%% Training and Test errors for the 20*20 Basis
train_error = zeros(size(trainX,2),1);
test_error = zeros(size(trainX,2),1);

for i = 1:size(trainX,2)
    A = A_train(1:i,:);
    y = trainy(1:i);
    w = (A'*A + lambda*eye(size(A',1)))\(A'*y);
    pred_testy = (A_test*w);
    test_error(i) = mean((pred_testy-testy).^2);    
    pred_trainy = (A*w);
    train_error(i) = mean((pred_trainy-y).^2);
end

%ploting the errors
figure();
truncate = 500;
plot((truncate:length(test_error)),test_error(truncate:length(test_error)), 'b');
hold on;
plot((truncate:length(test_error)),train_error(truncate:length(train_error)),'r');
xlabel('Number of Training Points');
ylabel('Error');
title('Error for Basis 20*20');
hold off;
legend('Test error','Training error');
