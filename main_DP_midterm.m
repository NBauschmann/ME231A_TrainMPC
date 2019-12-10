%% DP to precompute optimal speed profile (MIDTERM DATA)
% MPC Fall 2019
% Author: Nathalie Bauschmann
%
% Using system dynamics from midterm!
%
% Use Dynamic Programming Approach to precompute the optimal speed profile
% for each trian with the goal of traveling at the fastest allowed speed
% while satisfying speed constraints at any time.
% The DP finds the optimal velocity profile permitted by the speed
% limitations.
%
% Nearly same code as midterm, but using stage cost from paper
%
% Issues/Todo:
%  - try different gains for stage cost (defined in setup_parameters)
%         -> Kl_DP and Kv_DP
%  - Need to change final constraints and final cost
%     -> what should v_final be? 0?
%
%%
clear all; close all
%% Set path
addpath('data','functions')
%% Load parameters
load train_data_midterm

param = setup_parameters(1);  % overwrites param struct from midterm data 
                                 % 0 for paper, 1 for midterm data
A = param.A;
B = param.B;
C = param.C;
M = param.M;
g = param.g;

% Parameters for stage cost
Kv_DP = param.Kv_DP;
Kl_DP = param.Kl_DP;

%% Define state and input constraints
% Fmax0 = param.mumax*M*g; 
Fmax0 = 0.8 * param.mumax * M * g;  % testing
umin = -Fmax0; 
umax = Fmax0;

%% Grid the independent variable (same as gridding time)
track_length = profile(end,1); % in meters
ds = 10; % sampling in space
s_sampled = 0:ds:track_length; % sampled train position
N_s = length(s_sampled);

%% Grid the state space
v_min = 0.1;
dv = 0.1;
% grid from 0.1 to the max speed over the track at any s
v_sampled = v_min:dv:max(maxspeed(s_sampled)); 
N_v = length(v_sampled);
v_idx_set = 1:N_v;
%% Grid the input space
% dynamic input grid, umax and umin are calculated according to v for each
% step
N_u = 10;
%% This funtion computes the next speed given the current speed, input and position 
comp_v_next = @(v,u,s) v+ds/(v*M)*(-A-B*v-C*v^2-M*g*slope(s)-M*6/radius(s)+u); 

%% This funtion computes the input to bring speed v to speed v next 
% at position s
comp_u = @(v,v_next,s) M*(v_next-v)/(ds)*v-(-A-B*v-C*v^2-M*g*slope(s)-M*6/radius(s)); 

%% Define stage cost
% Jstage = @(v,u) ds/v;  % from midterm
Jstage = @(v,u,s) norm(v - maxspeed(s)) * Kv_DP + ...
    norm((v - maxspeed(s)) + abs(v - maxspeed(s))) * Kl_DP;

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

% initialize
JoptArray = zeros(length(v_idx_set),N_s-1);  % check if right size
UoptArray = zeros(length(v_idx_set),N_s-1);

tic
for s_indx = N_s-1:-1:1
    fprintf(' Solving DP at position = %i (meters) \n' ,s_sampled(s_indx));
    J(s_indx,:) = inf(1,N_v);
    u(s_indx,:) = nan(1,N_v);
    s = s_sampled(s_indx);
    for i = v_idx_set
        v = v_sampled(i);
        Jbest = inf; 
        Ubest = nan;
        um = comp_u(v,v_min,s);
        uM = comp_u(v,maxspeed(s+ds),s);
        for u_grid = linspace(max(umin,um),min(umax,uM),N_u)
            v_next = comp_v_next(v,u_grid,s);
            if v_next>=v_min && v_next<=maxspeed(s + ds)
                Jactual = Jstage(v,u_grid,s) + Jopt{s_indx+1}(v_next);
                if ~isnan(Jactual)
                    if Jactual<Jbest
                        Jbest = Jactual;
                        Ubest = u_grid;
                    end 
                end % ~isnan
            end % v inside constraints
        end % loop over u
        if v>maxspeed(s)  % should not be necessary
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

vOpt_DP = vOpt;
p_sampled = s_sampled;
% save date to DP_results.mat
save('data\DP_results.mat','vOpt_DP','p_sampled','uOpt')


%% DP to precompute optimal speed profile (MIDTERM DATA)
% MPC Fall 2019
% Author: Nathalie Bauschmann
%
% Using system dynamics from midterm!
%
% Use Dynamic Programming Approach to precompute the optimal speed profile
% for each trian with the goal of traveling at the fastest allowed speed
% while satisfying speed constraints at any time.
% The DP finds the optimal velocity profile permitted by the speed
% limitations.
%
% Nearly same code as midterm, but using stage cost from paper
%
% Issues/Todo:
%  - try different gains for stage cost (defined in setup_parameters)
%         -> Kl_DP and Kv_DP
%  - Need to change final constraints and final cost
%     -> what should v_final be? 0?
%
%  TODO: Actually save results -> differentiate between models
%
%%
clear all; close all
%% Set path
addpath('data','functions')
%% Load parameters
load train_data_midterm

param = setup_parameters(1);  % overwrites param struct from midterm data 
                                 % 0 for paper, 1 for midterm data
A = param.A;
B = param.B;
C = param.C;
M = param.M;
g = param.g;

% Parameters for stage cost
Kv_DP = param.Kv_DP;
Kl_DP = param.Kl_DP;

%% Define state and input constraints
Fmax0 = param.mumax*M*g; 
umin = -Fmax0; 
umax = Fmax0;

%% Grid the independent variable (same as gridding time)
track_length = profile(end,1); % in meters
ds = 10; % sampling in space
s_sampled = 0:ds:track_length; % sampled train position
N_s = length(s_sampled);

%% Grid the state space
v_min = 0.1;
dv = 0.1;
% grid from 0.1 to the max speed over the track at any s
v_sampled = v_min:dv:max(maxspeed(s_sampled)); 
N_v = length(v_sampled);
v_idx_set = 1:N_v;
%% Grid the input space
% dynamic input grid, umax and umin are calculated according to v for each
% step
N_u = 10;
%% This funtion computes the next speed given the current speed, input and position 
comp_v_next = @(v,u,s) v+ds/(v*M)*(-A-B*v-C*v^2-M*g*slope(s)-M*6/radius(s)+u); 

%% This funtion computes the input to bring speed v to speed v next 
% at position s
comp_u = @(v,v_next,s) M*(v_next-v)/(ds)*v-(-A-B*v-C*v^2-M*g*slope(s)-M*6/radius(s)); 

%% Define stage cost
% Jstage = @(v,u) ds/v;  % from midterm
Jstage = @(v,u,s) norm(v - maxspeed(s)) * Kv_DP + ...
    norm((v - maxspeed(s)) + abs(v - maxspeed(s))) * Kl_DP;

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

% initialize
JoptArray = zeros(length(v_idx_set),N_s-1);  % check if right size
UoptArray = zeros(length(v_idx_set),N_s-1);

tic
for s_indx = N_s-1:-1:1
    fprintf(' Solving DP at position = %i (meters) \n' ,s_sampled(s_indx));
    J(s_indx,:) = inf(1,N_v);
    u(s_indx,:) = nan(1,N_v);
    s = s_sampled(s_indx);
    for i = v_idx_set
        v = v_sampled(i);
        Jbest = inf; 
        Ubest = nan;
        um = comp_u(v,v_min,s);
        uM = comp_u(v,maxspeed(s+ds),s);
        for u_grid = linspace(max(umin,um),min(umax,uM),N_u)
            v_next = comp_v_next(v,u_grid,s);
            if v_next>=v_min && v_next<=maxspeed(s + ds)
                Jactual = Jstage(v,u_grid,s) + Jopt{s_indx+1}(v_next);
                if ~isnan(Jactual)
                    if Jactual<Jbest
                        Jbest = Jactual;
                        Ubest = u_grid;
                    end 
                end % ~isnan
            end % v inside constraints
        end % loop over u
        if v>maxspeed(s)  % should not be necessary
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
title('Results DP model from midterm')
subplot(3,1,1)
hold on; grid on
plot(s_sampled,maxspeed(s_sampled),s_sampled,vOpt)
legend('vmax','v DP')
xlabel('position (m)')
ylabel('velocity (m/s)')
subplot(3,1,2)
hold on; grid on
plot(s_sampled(1:end-1),uOpt)
xlabel('position (m)')
ylabel('Optimal Control (N)')
slopes = [];
subplot(3,1,3)
for s = s_sampled
    slopes = [slopes slope(s)];
end
hold on; grid on
plot(s_sampled,slopes)
xlabel('position (m)')
ylabel('slope (mm/m)')



