%% Linear Tangent Steering Problem
%
% Benchmarking Optimization Software with COPS
% Elizabeth D. Dolan and Jorge J. More
% ARGONNE NATIONAL LABORATORY
%
%% Problem Formulation
%
% Find u(t) over t in [0; t_F ] to minimize
%
% $$ J = t_f $$
%
% subject to:
%
% $$ \frac{d^{2}y_1}{dt^{2}} = a*cos(u) $$
% $$ \frac{d^{2}y_2}{dt^{2}} = a*sin(u) $$
% $$ |u| <= \frac{pi}{2} $$
%
% $$ y_{1:2}(0) = 0 $$
% $$ \frac{dy_{1:2}}{dt} = 0 $$
% $$ a = 1 $$
% $$ y_{2}(f) = 5 $$
% $$ \frac{dy_{1:2}}{dt}(f) = [45 \ 0] $$
%
% The following transformation gives a new formulation:
%
% $$ x_1 = y_1 $$
% $$ x_2 = \frac{dy_1}{dt} $$
% $$ x_3 = y_2 $$
% $$ x_4 = \frac{dy_2}{dt} $$
%
% $$ \frac{dx_1}{dt} = x_2 $$
% $$ \frac{dx_2}{dt} = a*cos(u) $$
% $$ \frac{dx_3}{dt} = x_4 $$
% $$ \frac{dx_4}{dt} = a*sin(u) $$

% Copyright (c) 2007-2008 by Tomlab Optimization Inc.

%% Problem setup
toms t
toms t_f
p = tomPhase('p', t, 0, t_f, 100);
setPhase(p);

tomStates x1 x2 x3 x4 m
tomControls u


m0 = 210;


% Initial guess
x0 = {t_f == 1
    icollocate({
    x1 == 12*t/t_f
    x2 == 45*t/t_f
    x3 == 5*t/t_f
    x4 == 0
    m == m0})};

% Box constraints
cbox = {sqrt(eps) <= t_f
    -pi/2 <= collocate(u) <= pi/2};

% Boundary constraints
cbnd = {initial({x1 == 0; x2 == 1.9976e+03; x3 == 3.2129e+05; x4 == 1.7316e+03})
    final({x3 == 400e3; x4 == 0})};

% ODEs and path constraints
Thrust = 700*2;
a = Thrust./m;
% a=100
ceq = collocate({dot(x1) == x2
    dot(x2) == a*cos(u)
    dot(x3) == x4
    dot(x4) == a*sin(u) - 6.674e-11.*5.97e24./(x3 + 6371e3).^2 + x2.^2./(x3 + 6371e3)
    dot(m) == -0.4744});
% Objective
objective = t_f;

%% Solve the problem
options = struct;
options.name = 'Linear Tangent Steering';
options.solver = 'knitro';
solution = ezsolve(objective, {cbox, cbnd, ceq}, x0, options);
t  = subs(collocate(t),solution);
x1 = subs(collocate(x1),solution);
x2 = subs(collocate(x2),solution);
x3 = subs(collocate(x3),solution);
x4 = subs(collocate(x4),solution);
m = subs(collocate(m),solution);
u  = subs(collocate(u),solution);

%% Plot result
subplot(2,1,1)
plot(t,x1,'*-',t,x2,'*-',t,x3,'*-',t,x4,'*-',t,m,'*-');
legend('x1','x2','x3','x4','m');
title('Linear Tangent Steering state variables');

subplot(2,1,2)
plot(t,u,'+-');
legend('u');
title('Linear Tangent Steering control');