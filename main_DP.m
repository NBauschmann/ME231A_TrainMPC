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

% ISSUES/TODO:
% - Do not have track data from paper, system params in paper and midterm
% are different -> atm param from midterm data is overwritten when
% parameters() is called
% - When (eventually) using paper track data: this contains stops! Careful
% with v_min in this case -> needs to be 0 at stations!
% (currently v_min = 0.1 as in midterm)
% - find sensible/efficient gridding for input space

clear all; close all
%% Set path
addpath('data','functions')
%% Load parameters
load train_data_midterm
parameters();   % overwrites param struct from midterm data 
%% Define state and input constraints
u_min = -param.M * param.abr;
u_max = param.M * param.adr;
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

%% Grid the input space for DP
%%%%%%%%%%%% What the midterm did:
% dynamic input grid, umax and umin are calculated according to v for each
% step
% N_u = 10;
% % This funtion computes the next speed given the current speed, input and position
% comp_v_next = @(v,u,s) v+ds/(v*M) *(-A-B*v-C*v^2-M*g*slope(s) -M*6/radius(s)+u);
% 
% % This funtion computes the input to bring speed v to speed v next
% at position s
% comp_u = @(v,v_next,s) M*(v_next - v)/(ds)*v - (-A-B*v-C*v^2-M*g*slope(s) -M*6/radius(s));
%%%%%%%%%%%%
% Cannot do this anymore -> find solution or just accept long runtimes
%%%%%%%%%%%%
N_u = 100;   % TODO: find sensible value
u_grid_values = linspace(u_min,u_max,N_u);

%% Define stage cost
Jstage = @(v,u) ds/v;

%% Initialization of Cost?to?Go
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
for p_indx = N_s-1:-1:1
    fprintf('Solving DP at position = %i (meters) \n',s_sampled(p_indx));
    J(p_indx,:) = inf(1,N_v);
    u(p_indx,:) = nan(1,N_v);
    s = s_sampled(p_indx);
    for i = v_idx_set
        v = v_sampled(i);
        Jbest = inf; 
        Ubest = nan;
        % um = comp_u(v,0.1,s);
        % uM = comp_u(v,maxspeed(s+ds),s);
        % for u_grid = linspace(max(umin,um),min(umax,uM),N_u)
        for u_grid = u_grid_values
            % v_next = comp_v_next(v,u_grid,s);
            F = 0;  % TODO
            x = [s; v; F];
            v_next = train_dynamics(x,u_grid,param);
            if v_next>=v_min && v_next<=maxspeed(s + ds)
                Jactual = Jstage(v,u_grid) + Jopt{p_indx+1}(v_next);
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
    JoptArray(i,p_indx) = Jbest;
    UoptArray(i,p_indx) = Ubest;
    end % loop over v
    Jopt{p_indx} = @(v) interpn(v_sampled,JoptArray(:,p_indx),v,'linear');
    Uopt{p_indx} = @(v) interpn(v_sampled,UoptArray(:,p_indx),v,'linear');
end % loop over p

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



