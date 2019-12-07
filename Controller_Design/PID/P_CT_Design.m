function [C, gains]= P_CT_Design(P, Wgc)
%[P,gains]= P_CT_Desgin(P, Wgc)
%
%P_CT_Desgin designs Continuous time Proportional controller using the unity crossover
%specification.
%
%   Inputs***
%   P = Plant transfer function
%   Wgc = unity gain crossover frequency in rad/s
%
%
%   Outputs***
%   gains(1, 2, 3, 4) = Kp, 0, 0, 0
%   P = P transfer function

% Vishwam Aggarwal
% Ver 1.0
% 12/06/2019

s = tf('s');

[M Ph] = bode(P, Wgc); % Calculate Magnitude and Phase at crossover for Plant and Controller Integrator

K = 1/M; % Compute Controller Gain

C = K; % Compute PI Controller Transfer Function

gains(1) = K; gains(2) = 0; gains(3) = 0; gains(4) = 0;
figure;
step(feedback(P*C,1))