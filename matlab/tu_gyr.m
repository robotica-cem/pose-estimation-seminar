function [x,P] = tu_gyr(x,P,gyr,Q,h)

q = x;
Sw = Somega(gyr);
F = expm(0.5*Sw*h);
q = F*q;

P = F*P*F' + Q;
x = q;

