clear;clc;close

% 1.    Fabian Domberg 
% 2.	Rakesh Reddy
% 3.	Tim-Henrik Traving
% 4.	Harsh Yadav

%Wine quality data set.
%--------------------------------------------------------------------------
%Distinguish between good wines and not good wines. 
%A wine is good if its score is 7 or higher.

%read data
load('Data.mat');

%TODO: Define X and y for the learning model


%TODO: Implement 5-fold Cross Validation. Iterate over the folds to split 
%the data X and y into X_train, y_train, X_test and y_test. 


%Example of nonlinear SVM training (fitcsvm) and prediction with trained
%model (predict). We use a radial basis function (rbf) kernel here.

C=1;
gamma=10;

n = length(Score);
rand_permutation = randperm(n);
X = SensorData(rand_permutation,:);
y = sign(Score(rand_permutation)-6.5);

num_folds = 5;
test_length = round(n/num_folds); 
acc = zeros(num_folds,1);

for fold = 1:num_folds 
    test_index = zeros(n,1);
    if fold ~= num_folds        
        for index = (fold-1)*test_length + 1: fold*test_length
            test_index(index) = 1;
        end
    else
        for index = n-test_length + 1 : n
            test_index(index) = 1;
        end
    end
    
    X_test = X(test_index == 1,:);
    y_test = y(test_index == 1);
    X_train = X(test_index == 0,:);
    y_train = y(test_index == 0);
    
    SVMModel_nonlinear=fitcsvm(X_train,y_train,'BoxConstraint',C,'KernelFunction','rbf','KernelScale',gamma);
    y_pred=predict(SVMModel_nonlinear,X_test);    
    acc(fold) = sum(y_pred==y_test)/length(y_test)*100; 
    fprintf('Prediction accuracy of %d fold is %2.1f\n', fold, acc(fold));    
end

mean_acc = mean(acc);
fprintf('Mean Prediction accuracy is %2.1f\n',mean_acc);

%Searching for the best Hyperparameter
disp('Optimising hyperparameters')

C=[1,10,100];
gamma=[0.1,1,10];
mean_acc = zeros(length(C),length(gamma));
max_mean_acc = 0;

n = length(Score);
rand_permutation = randperm(n);
X = SensorData(rand_permutation,:);
y = sign(Score(rand_permutation)-6.5);

num_folds = 5;
test_length = round(n/num_folds); 
acc = zeros(num_folds,1);

for a = 1:length(C)
    for b = 1:length(gamma)
        for fold = 1:num_folds 
            test_index = zeros(n,1);
            if fold ~= num_folds        
                for index = (fold-1)*test_length + 1: fold*test_length
                    test_index(index) = 1;
                end
            else
                for index = n-test_length + 1 : n
                    test_index(index) = 1;
                end
            end

            X_test = X(test_index == 1,:);
            y_test = y(test_index == 1);
            X_train = X(test_index == 0,:);
            y_train = y(test_index == 0);
            
            SVMModel_nonlinear=fitcsvm(X_train,y_train,'BoxConstraint',C(a),'KernelFunction','rbf','KernelScale',gamma(b));
            y_pred=predict(SVMModel_nonlinear,X_test);
            acc(fold) = sum(y_pred==y_test)/length(y_test)*100; 
        end

        mean_acc(a,b) = mean(acc); 
        if max_mean_acc < mean(acc)
            max_mean_acc = mean(acc);
            C_opt = C(a);
            gamma_opt = gamma(b);            
        end        
    end
end

fprintf('Max mean prediciton accuracy is %2.1f\n',max_mean_acc);
fprintf('Optimum value of C is %d\n', C_opt);
fprintf('Optimum value of Gamma is %2.1f\n', gamma_opt);







































