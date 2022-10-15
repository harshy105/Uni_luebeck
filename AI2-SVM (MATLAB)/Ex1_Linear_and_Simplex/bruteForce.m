function [v, class] = bruteForce(x, y, u)

v = zeros(size(x,1),1);

for i = 1:length(x(:,1))
    v(i) = norm(x(i,:)-u);
end

[~, index] = min(v);

class = y(index);

end