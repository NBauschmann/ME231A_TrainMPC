function [feas_f, xOpt_f, uOpt_f, JOpt_f] = cftoc_followingTrain(x0, ... 
     xbar, ubar, xleader, uoptleader, MODEL, param, slope_,radius_,limspeed_,maxspeed_)
%% solves CFTOC problem for the leading train
%% Inputs
% x0        initial state
% xbar      vector containing the estimated states, a priori estimatio
% ubar      vector containing the inputs used for the estimation
% xleader   leader state at time t (only first one needed!)
%           First scenario: the follower gets the leader state (s,v) at 
%           time t, if model==paper, F at time t is an estimation
%           xbar_l at time k+1 will then be computed by using the system
%           dynamics, where the input ubar_l will be the optimal input u at
%           time k, estimated at time t-1 -> need leader state from time
%           step before?
% uoptleader  leader optimal input at time t-1 to time t + Np - 2 
%           In theory, the follower would solve this optimization for the
%           follower for each time step as well, but we're saving time here
%           by handing it over
%           Will need to change this if introducing noise
%           
% MODEL     indicator which model is used  
% param     struct containing all the system + MPC parameters 
% slope_     
% radius_   
% limspeed_ 
% maxspeed_
% vOpt_DP   data from DP, optimal velocity at sample points
%% Outputs
% feas_l    information if CFTOC problem is feasible
% xOpt_l    optimal sequence of states 
% uOpt_l    optimal input sequence 
% Jopt_l    optimal cost function

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% for decision which MODEL is used
midterm = 1 ;
paper = 0 ;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%
if MODEL == midterm
%% extract paramters 
Kv_f = param.Kv_f; % gain for penalizing deviation from max speed
Kd = param.Kd;   % gain for penalizing deviation from desired distance
M = param.M;
Np = param.Np;  % MPC horizon 
g = param.g;
d_des = param.d_des;  % desired distance to leader
d_min = param.d_min;
%% define optimization variable for state
x = sdpvar(2,Np+1);
assign(x(:,1),x0);
%% define optimization variable for input 
u = sdpvar(1,Np);
%% define objective function
objective = 0;
% initialize distance to leader in each time step
d = zeros(Np);
xbar_l = zeros(2,Np);
xbar_l(:,1) = xleader;
for k = 1:Np   
    if k>1  % propagate leader states
        xbar_l(:,k) = train_dynamics_midterm(xbar_l(:,k-1),uoptleader(:,k));
    end
    d(k) = x(1,k) - xbar_l(1,k) - L;  % current distance to leader
    objective = objective + Kd * norm(d(k) - d_des) + ...
        Kv_f * norm(x(2,k) - maxspeed_(x(1,k)));   % used in paper  
end
%% define constraints
constraints = [];
for i = 1:Np
constraints = [constraints x(:,i+1) == train_dynamics_midterm(x(:,i),...
        u(1,i),param,slope_,radius_,limspeed_,maxspeed_)... %  with estimated values  alternatively: x(:,i+1) == train_dynamics(x(:,i), u(1,i), param) used in paper
    0 <= x(2,i+1) <= maxspeed_(xbar(1,i+1)) ...
    d_min <= d(i) ...
    -M * g * param.mumax  <= u(1,i) <= M * g * param.mumax]; 
    %-Pbr <= x(2,i+1) * u(1,i) <= Pdr];
end
% terminal constriant, from precomputed DP
constraints  = [constraints 0 <= x(2,Np+1) <= limspeed_(xbar(1,Np+1))];
%%
options = sdpsettings('verbose',1,'usex0',1,'solver','fmincon','fmincon.MaxIter',500000,...
    'fmincon.MaxFunEvals',500000);

sol = optimize(constraints, objective, options);

error = sol.problem;

if error == 0
    % correct solution found
    feas_f = 1;
    xOpt_f = double(x);
    uOpt_f = double(u);
    JOpt_f = double(objective);

else
    feas_f = 0;
    xOpt_f = [];
    uOpt_f = [];
    JOpt_f = [];
    
end % model == midterm
%%
% model selection
elseif MODEL == paper

% paramters 
delta_t = param.delta_t ;
Kv_l = param.Kv_l ;
Kj_l = param.Kj_l ;
M = param.M ;
Np = param.Np ;
jmax = param.jmax ;
abr = param.abr ;
adr = param.adr ; 
Pbr = param.Pbr ;
Pdr = param.Pdr ;


% define optimization variable for state
x = sdpvar(3,Np+1);
assign(x(:,1),x0);

% define optimization variable for input 
u = sdpvar(1,Np);

% define objective function
objective = 0 ;

for k = 1:Np
    % objective = objective + Kv_l * norm(x(2,k) - maxspeed(xbar(1,k)));     
    objective = objective + Kv_l * norm(x(2,k) - maxspeed_(x(1,k)));   % used in paper  
    % here the estimated state (position) is used... (deviation from
    % paper, eq. 7a
    % could function maxspeed would have to be adapted
end
for k = 1:Np-1
    objective = objective + Kj_l * norm((x(3,k+1) - x(3,k)) / (delta_t* M));     
end

% define constraints
constraints = [];
for i = 1:Np
constraints = [constraints x(:,i+1) == train_dynamics(x(:,i), u(1,i), param,slope_,radius_,limspeed_,maxspeed_)... %  with estimated values  alternatively: x(:,i+1) == train_dynamics(x(:,i), u(1,i), param) used in paper
    0 <= x(2,i+1) <= maxspeed_(xbar(1,i+1)) ...
    -jmax <= (x(3,k+1) - x(3,k))/ (M * delta_t) <= jmax ....
    -M * abr <= u(1,i) <= M * adr...
    -Pbr <= x(2,i+1) * u(1,i) <= Pdr];
end
constraints  = [constraints 0 <= x(2,Np+1) <= limspeed_(xbar(1,Np+1))];

options = sdpsettings('verbose',1,'usex0',1,'solver','fmincon','fmincon.MaxIter',500000,...
    'fmincon.MaxFunEvals',5000000);

sol = optimize(constraints, objective, options);

error = sol.problem;

if error == 0
    % correct solution found
    feas_f = 1;
    xOpt_f = double(x);
    uOpt_f = double(u);
    JOpt_f = double(objective);

else
    feas_f = 0;
    xOpt_f = [];
    uOpt_f = [];
    JOpt_f = [];
end  % model == paper
    
else
    error('MODEL is not assigned correctly')
end % model selection

end % function