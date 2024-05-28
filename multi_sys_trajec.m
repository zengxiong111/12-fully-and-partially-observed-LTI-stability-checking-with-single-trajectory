clear
%r is the spetral radius of A
%m is the output dimension
%n is the system state dimension
%p is the control input dimension
r = 0.99;
v = 1;
n = 3;      %n is the state dimension
m = 1;      %m is the output dimension
p = 1;      %p is the input dimension

length=1000;
0;
T=3*n;

% for i = 1:5
%     [A,B,C,D] = system_generation(r,v,m,n,p);
%     Ob = obsv(A,C);
%     Co = ctrb(A,B);
%     % check if the genrated system is controllable and observable
%     if(rank(Ob) == n & rank(Co) == n )
%         break;
%     end
%     if(rank(Ob) < n | rank(Co) < n )
%         continue;
%     end
% end

N=5;
A_all = zeros(n,n,N);
A_all(:,:,1) = [1.01 0 0;0 0.99 0;0 0 0.99];
A_all(:,:,2) = [0.99 1 0;0 0.99 1;0 0 0.99];
A_all(:,:,3) = [1 1 0;0 0.99 1;0 0 0.99];
A_all(:,:,4) = [0.99 0 0;0 0.99 0;0 0 0.99];
A_all(:,:,5) = [1.01 1 0;0 0.99 1;0 0 0.99];
 

%counterexample: Jordan block
%A = [1.003 0;0 0.9];
%A = [1.001 0;0 0.9];
%A = [0.99 1;0 0.99];
%A = [0.99 0;0 0.99];
%A = [0.99 1;0 0.99];
  A = [1.01 0 0;0 0.99 0;0 0 0.99];
%A = [0.99 1 0;0 0.99 1;0 0 0.99];
%A = [1 1 0;0 0.99 1;0 0 0.99];
%A = [0.99 0 0;0 0.99 0;0 0 0.99];
% A = [1.01 1 0;0 0.99 1;0 0 0.99];
% norm(A)
% vrho(A)
% eig(A)
n=size(A,1);
% eig(A'*A-eye(n));
% t=3;
% temp = (A')^t*(A'*A-eye(n))*A^t;
% svd(temp)
%  eig(temp);
% trace(temp)
% ones(n,1)'*temp*ones(n,1)
%A = [1 -1;1 1];
% eig(A^2)


%A = [0.9 0.9;0 0.5];
%A = [0.9 0;0 0.5];
B = zeros(n,p);
C = ones(m,n);
C(1)=1;
D = zeros(m,p);

Q = eye(n);
R = eye(p);

sigma_u_2 = 0;
% sigma_w_2 = 0.001;
% sigma_v_2 = 0.001;

sigma_w_2 = 0.01;
sigma_v_2 = 0.01;


x_0 = 10*ones(n,1);

X_all = zeros(n,length,N);

[U,X_all,Y] = multiple_trajectories_generation(length,A_all,B,C,D,sigma_u_2,sigma_w_2,sigma_v_2,x_0);

X_norm = zeros(length,N);

for i =1:length
    for j=1:N
        X_norm(i,j) = norm(X_all(:,i,j));
    end
end

% figure;
% hold on;
time_index = 1:length;
plot(time_index,X_norm(:,1),time_index,X_norm(:,2),...
    time_index,X_norm(:,3),time_index,X_norm(:,4),...
    '-x','LineWidth',3); % plot the tight bound eq (4) in the paper


%legend('||x(t)||_2','||x(t+1)||_2-||x(t)||_2')
legend('unstable','stable','marginally stable','stable','Location','best')
 grid on;
ax = gca;
ax.LineWidth = 2;
ax.GridLineStyle = '--';
ax.GridAlpha = 0.8;
lgd.FontSize = 18;
xlabel('time t','FontSize',18) ;
%ylabel('||x||_2','FontSize',18) ;
set(gca,'FontSize',20)

%Yingying's method
%Ka is the constant for nonlinear system

%norm(X(:,i)) > Ka * rho^i

%Ali's method
%For m_p,m_t,c_s, we need to check the stability of the closed-loop.
% for i=1:N
%     for j=1:N
%         m_a_temp = max([1,norm(A_t_all(:,:,i,j))]);
%         m_s_temp = max([1,norm(B_t_all(:,:,i,j)), norm(C_t_all(:,:,i,j))]);
%         if(m_a_temp > m_a)
%             m_a = m_a_temp;
%         end
%         if(m_s_temp > m_s)
%             m_s = m_s_temp;
%         end
%     end
% end
% 
% for i=1:N
%     for j=1:N
%         if(vrho(A_t_all(:,:,i,j))<1)
%             P = dlyap(A_t_all(:,:,i,j),C_t_all(:,:,i,j)'*C_t_all(:,:,i,j));
%             m_p_temp = norm(P);
%             m_t_temp = trace(P);
%             c_s_temp = 5*(sigma_w_2*trace(P)+sigma_u_2*...
%                 trace(B_t_all(:,:,i,j)'*P*B_t_all(:,:,i,j))+...
%                 sigma_v_2*n)*log(1/delta_p);
%             if(m_p_temp > m_p)
%                 m_p = m_p_temp;
%             end
%             if(m_t_temp > m_t)
%                 m_t = m_t_temp;
%             end
%             if(c_s_temp > c_s)
%                 c_s = c_s_temp;
%             end
%         end
%     end
% end
% 
% sigma_m = max([sigma_w_2,sigma_u_2,sigma_v_2]);
% c_e = max([1,1/log(1+epsilon_a)]);
% c_r = m_p * (22*epsilon_c^(-2)+1)*sigma_m^2*c_e;
% c_p = 2*max([1,m_p/(epsilon_c^2)]);
% 
% 
% 
% M_all(1) = 5 * m_p * log(1/delta_p);
% 
% %tau_all(1) = n + log(1/delta_p);
% tau_all(1) = int16(max([1600/9*log(1/delta_p),...
%    1/log(1+epsilon_a)*log(6400*epsilon_c/(9*sigma_w_2*delta_p)*(M_all(1) + ...
%    c_r *log(2/delta_p)*n^2 * m_a^(4*n) + c_s * log(1/delta_p) ))]));


