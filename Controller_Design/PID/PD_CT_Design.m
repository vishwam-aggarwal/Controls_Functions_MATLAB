function [PD,gains]= PD_CT_Design(P, Wgc, PM, tau)
%[PD,gains]= PD_CT_Desgin(P, Wgc, PM, tau)
%
%PD_CT_Desgin designs Continuous time PD controller using unity crossover
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
%   gains(1,2,3,4) = Kp, 0, Kd, tau
%   PD = PD transfer function

% Vishwam Aggarwal
% Ver 1.0
% 03/28/2019

s = tf('s');

[M Ph] = bode(P/(tau*s+1), Wgc); % Calculate Magnitude and Phase at crossover for Plant and Controller Integrator

z = Wgc/tand(PM-180-Ph) % Compute Controller Zero

K = 1/(M*sqrt(Wgc^2 + z^2)) % Compute Controller Gain

PD = K*(s + z)/(tau*s + 1); % Compute PI Controller Transfer Function

gains(1) = K*z; gains(2) = 0; gains(3) = K*(1-z*tau); gains(4) = tau;

step(feedback(P*PD,1))
end