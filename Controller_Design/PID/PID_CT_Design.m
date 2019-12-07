function [PID,gains]= PID_CT_Design(P, Wgc, PM, tau)
%[PID,gains]= PID_CT_Desgin(P, Wgc, PM, tau)
%
%PID_CT_Desgin designs Continuous time PID controller using unity crossover
%and Phase Margin specifications.
%
%   Inputs***
%   P = Plant transfer function
%   Wgc = unity gain crossover frequency in rad/s
%         If left empty [], then code will select it to be same as P bw
%   PM = Phase margin in degrees
%   tau = time constant for pseudo pole
%
%   Outputs***
%   gains(1,2,3,4) = Kp, Ki, Kd, tau
%   PID = PID transfer function

% Vishwam Aggarwal
% Ver 1.0
% 09/28/2018

s = tf('s');

[M Ph] = bode(P/(s*(tau*s+1)), Wgc); % Calculate Magnitude and Phase at crossover for Plant and Controller Integrator

z = Wgc/tand((PM-180-Ph)/2) % Compute Controller Zero

K = 1/(M*(Wgc^2 + z^2)) % Compute Controller Gain

PID = K*(s + z)^2/(s*(tau*s + 1)); % Compute PI Controller Transfer Function

gains(1) = 2*K*z-K*z^2*tau; gains(2) = K*z^2; gains(3) = K-2*K*z*tau+K*z^2*tau^2; gains(4) = tau;

step(feedback(P*PID,1))
end