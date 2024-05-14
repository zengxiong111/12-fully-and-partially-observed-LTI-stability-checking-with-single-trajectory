clear
syms r real
syms r1 r2 r3 real
% A = [r 1;0 r];
% M1 = A'*A-eye(2)
% t=2;
% M2 = (A')^t *(A'*A-eye(2))*A^t

% A = [r 1 0;0 r 1;0 0 r];
% M1 = A'*A-eye(3)
% t=2;
% M2 = (A')^t *(A'*A-eye(3))*A^t

%A = [r 0 0;0 r 0;0 0 r];
A = [r1 0 0;0 r2 0;0 0 r3];
M1 = A'*A-eye(3)
t=1;
M2 = (A')^t *(A'*A-eye(3))*A^t


% eig(M1)
% eig(M2)

% trace(M1)
% simplify(trace(M2))
n = size(A,1);
simplify(ones(n,1)'*M1*ones(n,1))
 simplify( ones(n,1)'*M2 * ones(n,1))
