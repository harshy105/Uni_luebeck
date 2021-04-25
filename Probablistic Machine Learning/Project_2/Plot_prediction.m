function [] = Plot_prediction(testX, testY, pred_testY)

figure();
scatter3(testX(:,1),testX(:,2),testY,'b','.');
hold on;
scatter3(testX(:,1),testX(:,2),pred_testY);
hold off;
xlabel('Latitute');
ylabel('Logitude');
zlabel('Temperature');
legend('Original Temperature','Predicted Temperature')

end