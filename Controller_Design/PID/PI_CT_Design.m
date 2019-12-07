function [PI,gains]= PI_CT_Design(P, Wgc, PM)
%[PI,gains]= PI_CT_Desgin(P, Wgc, PM)
%
%PI_CT_Desgin designs Continuous time PI controller using unity crossover
%and Phase Margin specifications.
%
%   Inputs***
%   P = Plant transfer function
%   Wgc = unity gain crossover frequency in rad/s
%         If left empty [], then code will select it to be same as P bw
%   PM = Phase margin in degrees
%
%   Outputs***
%   gains(1,2,3,4) = Kp, Ki,0,0
%   PI = PI transfer function

% Vishwam Aggarwal
% Ver 1.0
% 09/13/2018

s = tf('s');

[M Ph] = bode(P/s, Wgc); % Calculate Magnitude and Phase at crossover for Plant and Controller Integrator

z = Wgc/tand(PM-180-Ph); % Compute Controller Zero

K = 1/(M*sqrt(Wgc^2 + z^2)); % Compute Controller Gain

PI = K*(s + z)/s; % Compute PI Controller Transfer Function

K_p = K; K_i = z*K_p; % Decompose deliverables into PI Gains

gains = [K_p K_i 0 0]; % PI Gains

step(feedback(P*PI,1))