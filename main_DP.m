%% DP to precompute optimal speed profile
% MPC Fall 2019
% Author: Nathalie Bauschmann

% Using system dynamics from paper!

% Use Dynamic Programming Approach to precompute the optimal speed profile
% for each trian with the goal of traveling at the fastest allowed speed
% while satisfying speed constraints at any time.
% The DP finds the optimal velocity profile permitted by the speed
% limitations.

% Implemented according to "A model predictive control approach for virtual
% coupling in railways"
% Using system dynamics from the paper, this means given v(k) and v(k+1),
% we cannot use backwards computation of u(k) to decrease size of input
% space. (Because we cannot solve directly for u(k))
% This means we have to grid over the entire input space in the DP approach
%
% Should not be a problem though, because this file gets executed *only
% once* to save the optimal speed profile.
%
% ISSUES/TODO:
% - backwards computation of u given v and v_next based on trapezoidal
% formula from paper does not work yet.
% -> TODO: check reasoning behind trapezoidal formula, think of other ways
% to grid over u if this does not work
% - When (eventually) using paper track data: this contains stops! Careful
% with v_min in this case -> needs to be 0 at stations!
% (currently v_min = 0.1 as in midterm)
%
%  TODO: Actually save results -> differentiate between models
%

clear all; close all
%% Set path
addpath('data','functions')
%% Load parameters
load train_data_midterm

param = setup_parameters(0);  % overwrites param struct from midterm data 
                                 % 0 for paper, 1 for midterm data
A = param.A;
B = param.B;
C = param.C;
M = param.M;
g = param.g;
%% Define state and input constraints
u_min = -param.M * param.abr;
u_max = param.M * param.adr;
%% Grid the independent variable (same as gridding time)
track_length = profile(end,1); % in meters
ds = 10; % sampling in space
s_sampled = 0:ds:track_length; % sampled train position
N_s = length(s_sampled);
%% Grid the state space (v)
v_min = 0.1;
dv = 0.1;
% grid from 0.1 to the max speed over the track at any s
v_sampled = v_min:dv:max(maxspeed(s_sampled)); 
N_v = length(v_sampled);
v_idx_set = 1:N_v;

%% This function computes the next speed given the current speed, input and position
% -> Using trapezoidal formula to avoid gridding over v and F
% -> Only gridding over v, using:
%    v_i+1 = sqrt(v_i^2 + (2*delta_s*(-A -B*v_i - C*v_i^2 - Fg - u_i) / M ))
comp_v_next = @(v,u,s) real(sqrt(v^2 + 2*ds*(-A -B*v -C*v^2 + M*g*slope(s) - u) / M));
%% Grid the input space for DP
%%%%%%%%%%%% What the midterm did:
% dynamic input grid, umax and umin are calculated according to v for each
% step
% % This funtion computes the next speed given the current speed, input and position
% comp_v_next = @(v,u,s) v+ds/(v*M) *(-A-B*v-C*v^2-M*g*slope(s) -M*6/radius(s)+u);
% 
% % This funtion computes the input to bring speed v to speed v next
% at position s
% comp_u = @(v,v_next,s) M*(v_next - v)/(ds)*v - (-A-B*v-C*v^2-M*g*slope(s) -M*6/radius(s));
%% This function computes input to bring speed v to speed_v_next
% at position s
% Using trapezoidal formula used to discretize state
% space to solve for u:
Nu = 10;  % number of u grid points in dynamic input grid
comp_u = @(v,v_next,s)   A + B*v + C*v^2 - M*g*slope(s) - ...
    (v_next^2 - v^2)*M /(2*ds);
%% Define stage cost
Jstage = @(v,u) ds/v;

%% Initialization of Cost-to-Go
for i = v_idx_set
    if v_sampled(i) > maxspeed(s_sampled(N_s))
        J(N_s,i) = inf;
    else
        J(N_s,i) = ds/v_sampled(i);
    end
end
Jtogo = @(v) interpn(v_sampled,J(N_s,:),v,'linear');

Jopt{N_s} = @(v) 0;


%% Perform Dynamic Programming
% Optimization problem: 
% J*_i->Ns(v_i) = min u   q (v_i,u_i) + J*_i+1->Ns (v_i+1) 

% initialize
JoptArray = zeros(length(v_idx_set),N_s-1);  % check if right size
UoptArray = zeros(length(v_idx_set),N_s-1);

tic
for s_indx = N_s-1:-1:1
    fprintf('Solving DP at position = %i (meters) \n',s_sampled(s_indx));
    J(s_indx,:) = inf(1,N_v);
    u(s_indx,:) = nan(1,N_v);
    s = s_sampled(s_indx);
    for i = v_idx_set
        v = v_sampled(i);
        Fg = - M*g*slope(s);
        Jbest = inf; 
        Ubest = nan;
        % Find input needed to get to v_min and v_max to grid input space
        % optimally
        um = comp_u(v,v_min,s);
        uM = comp_u(v,maxspeed(s+ds),s);
        for u_grid = linspace(max(u_min,um),min(u_max,uM),Nu)
            v_next = comp_v_next(v,u_grid,s);
            if v_next>=v_min && v_next<=maxspeed(s + ds)
                Jactual = Jstage(v,u_grid) + Jopt{s_indx+1}(v_next);
                if ~isnan(Jactual)
                    if Jactual<Jbest
                        Jbest = Jactual;
                        Ubest = u_grid;
                    end 
                end % ~isnan
            end % v inside constraints
        end % loop over u
        if v>maxspeed(s)  % should not be necessary?
            Jbest = inf;
            Ubest = nan;
        end
    JoptArray(i,s_indx) = Jbest;
    UoptArray(i,s_indx) = Ubest;
    end % loop over v
    Jopt{s_indx} = @(v) interpn(v_sampled,JoptArray(:,s_indx),v,'linear');
    Uopt{s_indx} = @(v) interpn(v_sampled,UoptArray(:,s_indx),v,'linear');
end % loop over s

fprintf(' Total solution time: %i\n' , toc);

%% Simulate the system
% from initial condition v(0) = 0.1 and p(0) = 0

vOpt = zeros(1,N_s);
uOpt = zeros(1,N_s -1);
vOpt(1,1) = v_min;

for i = 1:N_s-1
    uOpt(i) = Uopt{i}(vOpt(:,i));
    vOpt(:,i+1) = comp_v_next(vOpt(:,i),uOpt(:,i),s_sampled(i));
end

% plotting
figure
subplot(3,1,1)
plot(s_sampled,maxspeed(s_sampled),s_sampled,vOpt)
legend('vmax','actual velocity')
subplot(3,1,2)
plot(s_sampled(1:end-1),uOpt)
xlabel('position')
ylabel('Optimal Control')
slopes = [];
subplot(3,1,3)
for s = s_sampled
    slopes = [slopes slope(s)];
end
plot(s_sampled,slopes)
xlabel('position')
ylabel('slope')

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%s
%% Debugging
figure
% plot of slope and maxspeed
slopes = [];
for s = s_sampled
    slopes = [slopes slope(s)];
end
subplot(2,1,1)
plot(s_sampled,slopes)
xlabel('position')
ylabel('slope')

maxspeeds = [];
for s = s_sampled
    maxspeeds = [maxspeeds maxspeed(s)];
end
subplot(2,1,2)
plot(s_sampled,maxspeeds)
xlabel('position')
ylabel('maxspeed')

%% Debugging
v = 20;
s = 1100; % slope is zero here, maxspeed is 23.6111
u = 0; % umin: -147936; umax: 144720
comp_u = @(v,v_next,s)   A + B*v + C*v^2 - M*g*slope(s) - ...
    (v_next^2 - v^2)*M /(2*ds);
comp_v_next = @(v,u,s) real(sqrt(v^2 + 2*ds*(-A -B*v -C*v^2 + M*g*slope(s) - u) / M));


v_next = comp_v_next(v,u,s);
u_backwards = comp_u(v,v_next,s);

%% 

comp_u_test = @(v_next) A + B*v + C*v^2 - M*g*slope(s) - (v_next.^2 - v^2)*M /(2*ds);

comp_v_next_test = @(u) sqrt(v^2 + 2*ds*(-A -B*v -C*v^2 + M*g*slope(s) - u) / M);

u = linspace(u_min, u_max, 1000);
v_next = linspace(0.1, 26, 1000);
figure
grid on; hold on
title(['comp v next for v=', num2str(v), ' , s=', num2str(s)])
% plot(-10e6:1000:10e6,comp_v_next_test(-10e6:1000:10e6))
plot(u,comp_v_next_test(u))
xlabel('u')
ylabel('v next')

figure
grid on; hold on
title(['comp u for v=', num2str(v), ' , s=', num2str(s)])
plot(v_next,comp_u_test(v_next))
xlabel('v next')
ylabel('u')
