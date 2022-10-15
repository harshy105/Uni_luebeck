clear; clc;close;
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


%Example of linear SVM training (fitcsvm) and prediction with trained
%model (predict). We use a radial basis function (rbf) kernel here.

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
    
    SVMModel_linear = fitcsvm(X_train,y_train);
    y_pred = predict(SVMModel_linear,X_test);    
    acc(fold) = sum(y_pred==y_test)/length(y_test)*100; 
    fprintf('Prediction accuracy of %d fold is %2.1f\n', fold, acc(fold));    
end

mean_acc = mean(acc);
fprintf('Mean Prediction accuracy is %2.1f\n',mean_acc);


