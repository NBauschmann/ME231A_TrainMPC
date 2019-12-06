%% Midterm 2017
% exercise 4

clear all
close all
clc

%% Dynamic Programming ? Train Control 
%% Author: Francesco Borrelli 2017(c) 
%% set up path
addpath('data','functions')
load train_data_midterm

%% Load parameters
M = param.M; 
g = param.g; 
A = param.A; 
B = param.B; 
C = param.C;

%% Define state and input constraints 
Fmax0 = param.mumax*M*g; 
umin = -Fmax0; 
umax = Fmax0;

%% Grid the indipendent variable (same as gridding time)  
track_length = profile(end,1); % in meters 
dp = 10; % sampling in space p 
p_sampled = 0:dp:track_length; %sampled train position 
N_p = length(p_sampled);

%% Grid the state space 
v_sampled = 0.1:0.1:max(maxspeed(p_sampled)); 

% grid from 0.1 to the max speed over the track at
N_v = length(v_sampled); 
v_idx_set = 1:N_v;

%% This funtion computes the next speed given the current speed, input and position 
comp_v_next = @(v,u,p) v+dp/(v*M)*(-A-B*v-C*v^2-M*g*slope(p)-M*6/radius(p)+u); 

%% This funtion computes the input to bring speed v to speed v next 
% at position 
p_comp_u = @(v,v_next,p) M*(v_next-v)/(dp)*v-(-A-B*v-C*v^2-M*g*slope(p)-M*6/radius(p)); 

%% Define stage 
cost_Jstage = @(v,u) dp/v;

%% Initialization of Cost?to?Go

for i = v_idx_set 
    if v_sampled(i) > maxspeed(p_sampled(N_p))
        J(N_p,i) = inf;
    else
        J(N_p,i) = dp/v_sampled(i);
    end
end
Jtogo = @(v) interpn(v_sampled,J(N_p,:),v,'linear');

Jopt{N_p} = @(v) 0;

%% Perform Dynamic Programming

tic
for p_idx = N_p-1:-1:1
    fprintf('Solving DP at position = %i (meters) \n',p_sampled(p_idx));
    J(p_idx,:) = inf(1,N_v);
    u(p_idx,:) = nan(1,N_v);
    
    p_ = p_sampled(p_idx);
    
    
    % iterate through states
    for v_idx = v_idx_set
        v_ = v_sampled(v_idx);
        
        % initialize best cost & best action
        Jbest = inf;
        Ubest = nan;
       
        % input that brings v to min speed of 0.1
        u_m = p_comp_u(v_,0.1,p_);
        
        % input that brings v to max speed dep on p_
        u_M = p_comp_u(v_,maxspeed(p_+dp),p_);
        
        % grid input space as described in exercise
        u_grid = linspace(max(u_m,umin),min(u_M,umax),10);
        l_u = length(u_grid);
        
        for u_idx = 1:l_u % iterate through input grid
            u_ = u_grid(u_idx);
            
            % compute next state v_next
            v_next = comp_v_next(v_,u_,p_);
            
            % check for feasibility of next state, input pair
            if v_next <= maxspeed(p_+dp) && v_next >= 0.1
                
                % Calculate cost-to-go at current state using
                % current input
                J_ = cost_Jstage(v_,u_) + Jopt{p_idx + 1}(v_next);
                if ~isnan(J_) 
                    if J_< Jbest
                        Jbest = J_;
                        Ubest = u_;
                    end
                end     
            end
        end
        
        if v_> maxspeed(p_)
           Jbest = inf;
           Ubest = nan;
        end
        J(v_idx,p_idx) = Jbest;
        u(v_idx,p_idx) = Ubest;
    
    end
    Jopt{p_idx} = @(v) interpn(v_sampled,J(:,p_idx),v,'linear');
    uopt{p_idx} = @(v) interpn(v_sampled,u(:,p_idx),v,'linear');
    
end
fprintf('Total solution time: %i\n',toc); 

%% Simulate the system from the initial condition v(0)=0.1 and p(0)=0 and plot ... 


vOpt = zeros(1,N_p);
uOpt = zeros(1,N_p -1);
vOpt(1,1) = 0.1;

for i = 1:N_p-1
    uOpt(i) = uopt{i}(vOpt(:,i));
    vOpt(:,i+1) = comp_v_next(vOpt(:,i),uOpt(:,i),p_sampled(i));
end

%% plots 
subplot(3,1,1)
plot(p_sampled,vOpt)
hold on
plot(p_sampled,maxspeed(p_sampled))

subplot(3,1,2)
plot(p_sampled(1:end-1),uOpt)

slope_ = [];
for i = 1:N_p-1
    slope_ = [slope_; slope(p_sampled(i))];
end

subplot(3,1,3)
plot(p_sampled(1:end-1),slope_)






