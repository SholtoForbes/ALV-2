% ALV-2 Trajectory Simulation
% Sholto Forbes-Spyratos

% optimise 
A = [];
b = [];
Aeq = [];
beq = [];

lb = [-.1,-10];
ub = [.1,10];

x0 = [0,0];

% lb = [-0.0001, -.001,-.01,-.1,-10];
% ub = [0.0001, .001,.01,.1,10];
% 
% x0 = [0,0,0,0,0];

nonlcon = [];
options = optimoptions('fmincon','Display','iter','Algorithm','sqp');


x = fmincon(@ALV2FUNCTION,x0,A,b,Aeq,beq,lb,ub,nonlcon, options)  % run trajectory calculation routine, this determines the correct angle of attack schedule for the upper stages

[rdiff,t,r,gamma,v,m] = ALV2FUNCTION(x); % simulate trajectory

r_E = 6371000; % radius of Earth (m)

figure(1)

subplot(4,1,1)
plot(t,(r-r_E)/1000)
subplot(4,1,2)
plot(t,rad2deg(gamma))
subplot(4,1,3)
plot(t,v)
subplot(4,1,4)
plot(t,m)
