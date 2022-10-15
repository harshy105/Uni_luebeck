function K = periodic_kernel(X1, X2, sigma, l, p)

K = zeros(length(X1),length(X2));
for i = 1:length(X1)
    for j = 1:length(X2)
        K(i,j) = sigma^2*exp((-2*sin(pi*norm(X1(i,:)-X2(j,:))/p)^2)/l^2);
    end
end

end
