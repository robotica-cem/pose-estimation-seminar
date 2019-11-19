function [x,P] = mu_acc(x,P,acc,Ra,g0, tol, normalize)

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
% Kalman gain
K = P*H'*inv(S);
% Updated estimate
qup = q + K*ytilde;
% Renormalize
qup = qup / norm(qup);

% Updated error cov
P = (eye(4) - K*H)*P;
x = qup;
