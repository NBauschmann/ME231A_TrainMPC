function [feas_l, xOpt_l, uOpt_l, JOpt_l] = cftoc_leadingTrain(x0, ... 
     xbar, ubar, MODEL, param, p_sampled, vOpt_DP)
%% solves CFTOC problem for the leading train
%% Inputs
% ubar      vector containing the inputs used for the estimation
% xbar      vector containing the estimated states, a priori estimation
% MODEL     indicator which model is used  
% param     struct containing all the system + MPC parameters 
% p_sampled data from DP, position sampling points 
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


if MODEL == midterm
    
    % define optimization variable for state
    x = sdpvar(2,Np+1);
    assign(x(:,1),x0);
    
    % define optimization variable for input 
    u = sdpvar(1,Np);
    
    % define objective function
    objective = 0 ;
    
    for k = 1:Np
        objective = objective + norm(x(2,k) - maxspeed(x(1,k)));     
    end
       
    
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
        objective = objective + Kv_l * norm(x(2,k) - maxspeed(x(1,k)));   % used in paper  
        % here the estimated state (position) is used... (deviation from
        % paper, eq. 7a
        % could function maxspeed would have to be adapted
    end
    for k = 1:Np-1
        objective = objective + Kj_l * norm((x(3,k+1) - x(3,k))* M / delta_t);     
    end
    
    % define constraints
    constraints = [];
    for i = 1:Np
    constraints = [constraints x(:,i+1) == train_dynamics(x(:,i), u(1,i), param)... %  with estimated values  alternatively: x(:,i+1) == train_dynamics(x(:,i), u(1,i), param) used in paper
        0 <= x(2,i+1) <= maxspeed(xbar(2,i+1)) ...
        -jmax <= (x(3,k+1) - x(3,k))/ (M * delta_t) <= jmax ....
        -M * abr <= u(1,i) <= M * adr...
        -Pbr <= x(2,i+1) * u(1,i) <= Pdr];
    end
    constraints  = [constraints 0 <= x(2,Np+1) <= v_DP_l(xbar(2,Np+1),p_sampled ,vOpt_DP)];
    
    options = sdpsettings('verbose',1,'usex0',1,'solver','fmincon','fmincon.MaxIter',500000,...
        'fmincon.MaxFunEvals',500000);
    
    sol = optimize(constraints, objective, options);
 
    error = sol.problem;
    
    if error == 0
        % correct solution found
        feas_l = 1;
        xOpt_l = double(x);
        uOpt_l = double(u);
        JOpt_l = double(objective);
    
    else
        feas_l = 0;
        xOpt_l = [];
        uOpt_l = [];
        JOpt_l = [];
    end 
    
else
    error('MODEL is not assigned correctly');
end

end