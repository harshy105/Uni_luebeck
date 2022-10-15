%Doubt
X = [7 5 4 -4 0 -2 -1;
    -1 -3 1 -2 3 2 -3];
n = 2;
A = [-2 0; 0 -1];

cvx_begin sdp
    variable P(n,n) symmetric;
    variable xc(n,1)
    max(det_inv(P));
    subject to
    (X(:,1)-xc)'*P*(X(:,1)-xc) <= 1;
cvx_end