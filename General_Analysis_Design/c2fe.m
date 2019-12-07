function [Pd] = c2fe(P, Ts)
%Pd = c2be(P, Ts)
%
%cc2be converts any continuous plant (P) into a discrete transfer function (Pd)
%using the forward Euler method for discretization for the given sample
%time (Ts)
%
% Vishwam Aggarwal
% Ver 1.0
% 05/25/2019
[num, den] = tfdata(P, 'vpa');

syms S Z
NUM = poly2sym(num, S);
DEN = poly2sym(den, S);

TF = subs(NUM/DEN, S, (Z-1)/Ts);
[N,D] = numden(TF);

n = double(coeffs(N, Z, 'All'));
d = double(coeffs(D, Z, 'All'));

Pd = tf(n, d, Ts);
Pd = minreal(Pd);
end