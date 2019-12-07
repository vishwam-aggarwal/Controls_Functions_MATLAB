 function sys_out = ss2cc(sys_in)
%sys_out = ss2cc(sys_in)
%
%ss2cc converts any state space representation (sys_in) into its controllable
%canonical form (sys_out)
%
% Vishwam Aggarwal
% Ver 1.0
% 05/21/2019

P = tf(sys_in);

[num,den] = tfdata(P, 'v');

n = order(P);

A_cc = eye(n-1,n-1);
A_cc = [zeros(n-1,1) , A_cc];
A_cc = [A_cc; zeros(1,n)];

for i=1:n
    A_cc(n,i) = -den(n-i+2);
end

B_cc = zeros(n,1);
B_cc(n,1) = 1;

C_cc = zeros(1,n);

for i=1:n
    C_cc(1,i) = num(n-i+2);
end

D_cc = 0;
[A,B,C,D,Ts] = ssdata(sys_in);
sys_out = ss(A_cc, B_cc, C_cc, D_cc, Ts);
