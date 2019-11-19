function [x,P] = mu_acc_robust(x,P,acc,Ra,g0, tol, normalize)

lambda = 2;
if abs(norm(g0) - norm(acc)) > tol
    x=x;
    P=P;
    return
end

if nargin > 6
    if normalize
        g0 = g0/norm(g0);
        acc = acc/norm(acc);
    end
end

q = x;
% Innovation
ytilde = acc - Qq(q)'*g0;
% Innovation cov
[Q0, Q1, Q2, Q3] = dQqdq(q);
H = [Q0'*g0 Q1'*g0 Q2'*g0 Q3'*g0];
S = H*P*H' + Ra;

ZP = sqrtm(inv(P));
ZR = sqrtm(inv(Ra));
        
% Minimize using cvx 
n = 4;
b = cat(1, ZR*ytilde, zeros(n,1));
A = cat(1, ZR*H, ZP);

cvx_begin quiet
   variable xtilde(n)
   minimize( sum( huber( A*xtilde-b , lambda) ) )
cvx_end

% Filter update
x = x + xtilde;
% Renormalize
x = x / norm(x);
      

% Kalman gain
K = P*H'*inv(S);

% Updated error cov
P = (eye(4) - K*H)*P;
