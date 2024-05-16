function [U_single,X,Y_single] = multiple_trajectories_generation(length,A_all,B,C,D,sigma_u_2,sigma_w_2,sigma_v_2,x_0)

n = size(A_all,1);
p = size(B,2);
m = size(C,1);

N = size(A_all,3);

U_single = mvnrnd(zeros(p,1),sigma_u_2*eye(p),length);
U_single = U_single';
W = mvnrnd(zeros(n,1),sigma_w_2*eye(n),length);
W = W';
Z = mvnrnd(zeros(m,1),sigma_v_2*eye(m),length);
Z = Z';

Y_single = zeros(m,length,N);
X = zeros(n,length,N);

%X(1,1:n) = normrnd(zeros(n,1),sigma_x0*eye(n),n,1)';
%X(1,1:n) = zeros(n,1);
for i=1:N
    X(:,1,i) = x_0;
end


for i=2:length
    for j=1:N
        X(:,i,j) = A_all(:,:,j) * X(:,i-1,j) + B * U_single(:,i-1) + W(:,i-1);
        Y_single(:,i-1,j) = C * X(:,i-1,j) + D * U_single(:,i-1) + Z(:,i-1);
    end 
end


 

end