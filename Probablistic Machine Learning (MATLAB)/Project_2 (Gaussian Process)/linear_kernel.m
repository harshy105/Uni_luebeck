function K = linear_kernel(X1, X2, sigma, sigma_b, c)

K = zeros(length(X1),length(X2));
for i = 1:length(X1)
    for j = 1:length(X2)
        K(i,j) = sigma_b^2 + sigma^2*(X1(i,:)-c)*(X2(j,:)-c)';
    end
end

end