clear
clc
%% c
N = 100;
x = unifrnd(-2,2,N,1);
w = normrnd(0,1,6,1); %w = a
eps = normrnd(0,100,N,1);
X = [ones(N,1) x x.^2 x.^3 x.^4 x.^5];
y = X*w + eps;

cvx_begin
    variable w_c(6)
    minimize (norm(X*w_c-y,2));
cvx_end
    
%% d
cvx_begin
    variable w_d(6)
    minimize (norm(X*w_d-y,2));
    subject to
    -0.5 <= w_d(5) <= 0.5;
    -0.5 <= w_d(6) <= 0.5;
cvx_end

hold off;
scatter(x,y,'x');

hold on;
x_plot = -2:0.1:2;
plot(x_plot,polyval(flip(w_c),x_plot),'r');
plot(x_plot,polyval(flip(w_d),x_plot),'g');

%% e
gamma = 1;

cvx_begin
    variable w_e(6)
    minimize (norm(X*w_e-y,2) + gamma*norm(w_e,1));
cvx_end

plot(x_plot,polyval(flip(w_e),x_plot),'b');

