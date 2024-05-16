function [U_single,X,Y_single] = single_trajectory_generation(N,A,B,C,D,sigma_u_2,sigma_w_2,sigma_v_2,x_0)

n = size(A,1);
p = size(B,2);
m = size(C,1);

U_single = mvnrnd(zeros(p,1),sigma_u_2*eye(p),N);
U_single = U_single';
W = mvnrnd(zeros(n,1),sigma_w_2*eye(n),N);
W = W';
Z = mvnrnd(zeros(m,1),sigma_v_2*eye(m),N);
Z = Z';

Y_single = zeros(m,N);
X = zeros(n,N);

%X(1,1:n) = normrnd(zeros(n,1),sigma_x0*eye(n),n,1)';
%X(1,1:n) = zeros(n,1);
X(:,1) = x_0;

for i=2:N
    X(:,i) = A * X(:,i-1) + B * U_single(:,i-1) + W(:,i-1);
    Y_single(:,i-1) = C * X(:,i-1) + D * U_single(:,i-1) + Z(:,i-1);
end


 

end