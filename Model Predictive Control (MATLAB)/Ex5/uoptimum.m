function u_optimum = uoptimum(A,B,Gu,hu,Gx,hx,P,Q,R,N,x0)

m = size(B,1);
n = size(B,2);
A_bar = zeros(m*N,m);
B_bar = zeros(m*N,N);
Q_bar = zeros(m*N,m*N);
R_bar = zeros(N,N);

for i = 0:N-1
    % compute A_bar
    A_bar(m*i+1:m*(i+1),:) = A^(i+1);
    
    % compute Q_bar
    if i < N-1
        Q_bar(m*i+1:m*(i+1),m*i+1:m*(i+1)) = Q;
    else
        Q_bar(m*i+1:m*(i+1),m*i+1:m*(i+1)) = P;
    end
    
    % compute R_bar
    R_bar(n*i+1:n*(i+1),n*i+1:n*(i+1)) = R;
    
    % compute B_bar
    for j = 0:i
        B_bar(m*i+1:m*(i+1),n*j+1:n*(j+1)) = A^(i-j)*B;        
    end
end

H_bar = B_bar'*Q_bar*B_bar + R_bar;
F_bar = (A_bar'*Q_bar*B_bar)'; %F_bar from tutorial defination is F_bar' from slides slides defination 


Gu_bar = kron(eye(N),Gu);
hu_bar = kron(ones(N,1),hu);
Gx_bar = kron(eye(N),Gx);
hx_bar = kron(ones(N,1),hx);

G_bar = [Gu_bar; Gx_bar*B_bar];
h_bar = [hu_bar; (hx_bar - Gx_bar*A_bar*x0)];

u_optimum = quadprog(H_bar, F_bar*x0, G_bar, h_bar);
end