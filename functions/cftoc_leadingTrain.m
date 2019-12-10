function [feas_l, xOpt_l, uOpt_l, JOpt_l] = cftoc_leadingTrain(x0, ... 
     xbar, ubar, MODEL, param, slope_,radius_,DPspeed_,maxspeed_)
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

    % paramters 
    Kv_l = param.Kv_l ;
    M = param.M ;
    Np = param.Np ;
    g = param.g ;


    
    % define optimization variable for state
    x = sdpvar(2,Np+1);
    %assign(x(:,1),x0);
    assign(x,xbar);

    
    % define optimization variable for input 
    u = sdpvar(1,Np);
    assign(u,ubar);

    
    % define objective function
    objective = 0 ;
    
    for k = 1:Np+1
        % objective = objective + Kv_l * norm(x(2,k) - maxspeed_(xbar(1,k)));     
        objective = objective + Kv_l * norm(x(2,k) - maxspeed_(x(1,k)));   % used in paper  
        % here the estimated state (position) is used... (deviation from
        % paper, eq. 7a
        % could function maxspeed would have to be adapted
    end
    
    % define constraints
    constraints = [x(:,1) == x0];
    for i = 1:Np
    constraints = [constraints;
                   x(:,i+1) == train_dynamics_midterm(x(:,i), u(1,i), param,slope_,radius_,DPspeed_,maxspeed_); %  with estimated values  alternatively: x(:,i+1) == train_dynamics(x(:,i), u(1,i), param) used in paper
                   0 <= x(2,i+1) <= maxspeed_(xbar(1,i+1)); % normally maxspeed
                   -M * g * param.mumax  <= u(1,i) <=   M * g * param.mumax]; 
                   % -Pbr <= x(2,i+1) * u(1,i) <= Pdr];
    end
    constraints  = [constraints;
                    0 <= x(2,Np+1) <= DPspeed_(xbar(1,Np+1))];

        options = sdpsettings('verbose',0,'usex0',1,'solver','fmincon','fmincon.MaxIter',500000,...
        'fmincon.MaxFunEvals',5000000,'fmincon.TolFun',1e-05,'fmincon.TolFunValue',1e-05);
    
    sol = optimize(constraints, objective, options);
 
    error = sol.problem;
    
    if error == 0
        % correct solution found
        feas_l = 1;
        xOpt_l = double(x);
        uOpt_l = double(u);
        JOpt_l = double(objective);
    
    else
        disp('The following error code occured (leading train)')
        disp(error)
        feas_l = 0;
        xOpt_l = [];
        uOpt_l = [];
        JOpt_l = [];
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
    assign(x,xbar);
    
    % define optimization variable for input 
    u = sdpvar(1,Np);
    assign(u,ubar);

    
    % define objective function
    objective = 0 ;
    
    for k = 1:Np+1
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
    constraints = [x(:,1) == x0];
    for i = 1:Np
    constraints = [constraints x(:,i+1) == train_dynamics(x(:,i), u(1,i), param,slope_,radius_,DPspeed_,maxspeed_); %  with estimated values  alternatively: x(:,i+1) == train_dynamics(x(:,i), u(1,i), param) used in paper
                   0 <= x(2,i+1) <= maxspeed_(xbar(1,i+1));
                   -jmax <= (x(3,k+1) - x(3,k))/ (M * delta_t) <= jmax;
                   -M * abr <= u(1,i) <= M * adr;
                   -Pbr <= x(2,i+1) * u(1,i) <= Pdr];
    end
    constraints  = [constraints;
                    0 <= x(2,Np+1) <= DPspeed_(xbar(1,Np+1))];
    
    options = sdpsettings('verbose',0,'usex0',1,'solver','fmincon','fmincon.MaxIter',500000,...
        'fmincon.MaxFunEvals',5000000,'fmincon.TolFun',1e-06,'fmincon.TolFunValue',1e-06);
    
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
    error('MODEL is not assigned correctly')
end

end