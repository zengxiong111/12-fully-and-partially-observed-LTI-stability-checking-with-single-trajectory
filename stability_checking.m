clear
%r is the spetral radius of A
%m is the output dimension
%n is the system state dimension
%p is the control input dimension
r = 0.9;
v = 1;
n = 2;      %n is the state dimension
m = 1;      %m is the output dimension
p = 1;      %p is the input dimension

N=20;
T=3*n;

for i = 1:5
    [A,B,C,D] = system_generation(r,v,m,n,p);
    Ob = obsv(A,C);
    Co = ctrb(A,B);
    % check if the genrated system is controllable and observable
    if(rank(Ob) == n & rank(Co) == n )
        break;
    end
    if(rank(Ob) < n | rank(Co) < n )
        continue;
    end
end

%counterexample: Jordan block
%A = [1.03 0.1;0 0.5];
svd(A'*A-eye(n))
t=10;
temp = (A')^t*(A'*A-eye(n))*A^t;
% svd(temp)
% eig(temp)
trace(temp)
ones(n,1)'*temp*ones(n,1)
%A = [1 -1;1 1];

% eig(A^2)


%A = [0.9 0.9;0 0.5];
%A = [0.9 0;0 0.5];
B = zeros(n,p);
C = zeros(m,n);
D = zeros(m,p);

Q = eye(n);
R = eye(p);

sigma_u_2 = 0;
sigma_w_2 = 0.00;
sigma_v_2 = 0.00;

x_0 = ones(n,1);

[U,X,Y] = single_trajectory_generation(N,T,A,B,C,D,sigma_u_2,sigma_w_2,sigma_v_2,x_0);

N_hat = size(X,2);
time_index = 1:N_hat;

X_norm = zeros(N_hat,1);
X_diff_norm = zeros(N_hat,1);

Y_norm = zeros(N_hat,1);

for i =1:N_hat-1
    X_diff_norm(i) = norm(X(:,i+1)) - norm(X(:,i));
end

X_diff_norm(N_hat) = X_diff_norm(N_hat-1);

for i =1:N_hat
    X_norm(i) = norm(X(:,i));
    Y_norm(i) = norm(Y(:,i));
end

% figure;
% hold on;
%plot(time_index,X_norm,time_index,X_diff_norm,'-x','LineWidth',3); % plot the tight bound eq (4) in the paper
plot(time_index,X_norm,'-x','LineWidth',3); % plot the tight bound eq (4) in the paper
%plot(time_index,X_diff_norm,'-x','LineWidth',3); % plot the tight bound eq (4) in the paper

 %legend('robustness margin from Hinf','LMI sufficient [0,+m]','LMI sufficient [-m,+m]','controllability margin')
%legend('||x(t)||_2','||x(t+1)||_2-||x(t)||_2')
legend('||x(t)||_2')
%legend('||x(t+1)||_2-||x(t)||_2')
grid on;
ax = gca;
ax.LineWidth = 2;
ax.GridLineStyle = '--';
ax.GridAlpha = 0.8;
lgd.FontSize = 18;
xlabel('time t','FontSize',18) ;
%ylabel('||x||_2','FontSize',18) ;
set(gca,'FontSize',20)


