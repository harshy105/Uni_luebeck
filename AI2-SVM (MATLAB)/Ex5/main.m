clear all; 
close all;
clc;

x1 = linspace(-3,3);
x2 = linspace(-3,3);
[X1,X2] = meshgrid(x1,x2);
f = X1.^2 + X2.^2 + 1;
g = X1 + X2 + 2*sqrt(2);

hold on;
contour(X1,X2,f,1:10);
contour(X1,X2,g,[0 0]);
plot(0,0,'.b');
axis equal;
grid minor;
xlabel('x_1')
ylabel('x_2')
legend('f(x_1,x_2)=c', 'g(x_1,x_2)=0')
hold off;

saveas(gcf,'main_plot.pdf')