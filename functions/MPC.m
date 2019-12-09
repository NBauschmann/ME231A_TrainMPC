function [feas, xOpt, uOpt, predErr, x_pred_l, x_pred_f, u_pred_l, u_pred_f] = MPC(x0_l, x0_f,param,MODEL,...
        slope_,radius_,DPspeed_,maxspeed_,p_sampled)
    
%% performs MPC
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Inputs 
% x0_l          initial position of leading train
% x0_f          initial position of following train
% param         struct with param
% MODEL         indicator which model is used
% slope_        function handle for slope function
% radius_       function handle for radius_ function
% limspeed_     function handle for limspeed_ function
% maxspeed_     function handle for maxspeed_ function
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Outputs
% feas       vector of logicals that indicate the feasibility of the problem at each simulation step
% xOpt       array of the optimal closed-loop trajectory 
% uOpt       array of the optimal input sequence 
% predErr    array of the l2-norm of the difference between the openloop predictions 
%            and the closed-loop trajectory for each state from simulation timestep 
% x_pred_l   array of predicted trajectories for leading train
% x_pred_f   array of predicted trajectories for following train
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 

% midterm = 1 ;
% paper = 0 ;
% 
% if MODEL == midterm
%     % number of states
%     nx = 2 ;
% elseif MODEL == paper
%     % number of states
%     nx = 3 ;
% else
%     error('MODEL is not assigned correctly')
% end
% ( currently not needed ) 

% turns off fmincon warning
warning('off','MATLAB:nearlySingularMatrix')


% initialisation of storage vectors
feas_l = [] ;
feas_f = [] ;

xOpt_l = x0_l;
xOpt_f = x0_f;

uOpt_l = [] ;
uOpt_f = [] ;

x_pred_l = [] ;
x_pred_f = [] ;
u_pred_l = [] ;
u_pred_f = [] ;

% predErr_l = zeros(nx,1);
% predErr_f = zeros(nx,1);

% initialisation for apriori estimation before first step
uOpt_l_t = 1.8449e+05*ones(1,param.Np) ;
uOpt_f_t = 1.8449e+05*ones(1,param.Np) ;

% indicator of MPC iteration
i = 1 ;



while xOpt_l(1,end) < p_sampled(1,end)
    tic
    % apriori estimation
    [xbar_l, ubar_l] = a_priori_estimation(x0_l, uOpt_l_t, param, MODEL, ...
        slope_, radius_, DPspeed_, maxspeed_);
    [xbar_f, ubar_f] = a_priori_estimation(x0_f, uOpt_f_t, param, MODEL, ...
        slope_, radius_, DPspeed_, maxspeed_);
    
    % execute finite time horizon optimization
    [feas_l_t, xOpt_l_t, uOpt_l_t, JOpt_l_t] = cftoc_leadingTrain(x0_l, ... 
       xbar_l, ubar_l, MODEL, param, slope_,radius_,DPspeed_,maxspeed_) ;
    
   if feas_l_t ~= 1
        xOpt_l = [];
        xOpt_f = [];
        uOpt_l = [];
        uOpt_f = [];
%         predErr_l = [];
%         predErr_f = [];
        feas_l = [feas_l, false]; 
        disp('No feasible solution found for leading train.')
        break;
   else
       feas_l = [feas_l, true];
       % store current prediction
       x_pred_l{i} = xOpt_l_t ;
       u_pred_l{i} = uOpt_l_t ;


        % execute finite time horizon optimization for following train
        [feas_f_t, xOpt_f_t, uOpt_f_t, JOpt_f_t] = cftoc_followingTrain(x0_f, ... 
            xbar_f, ubar_f, xOpt_l_t(:,1), uOpt_l_t, MODEL, param, slope_,radius_,DPspeed_,maxspeed_) ;
 
       if feas_f_t ~= 1
            xOpt_l = [];
            xOpt_f = [];
            uOpt_l = [];
            uOpt_f = [];
%             predErr_l = [];
%             predErr_f = [];
            feas_f = [feas_f, false];
            disp('No feasible solution found for following train.')
            break;
       else
            feas_f = [feas_f, true];
            % optimal closed loop trajectory
            xOpt_l = [xOpt_l, xOpt_l_t(:,2)];
            xOpt_f = [xOpt_f, xOpt_f_t(:,2)];
            uOpt_l = [uOpt_l, uOpt_l_t(1,1)];
            uOpt_f = [uOpt_f, uOpt_f_t(1,1)];
            % new initial position
            x0_l = xOpt_l_t(:,2) ;
            x0_f = xOpt_f_t(:,2) ;
            % store current prediction
            x_pred_f{i} = xOpt_f_t ;
            u_pred_f{i} = uOpt_f_t ;


       end
    disp('MPC iteration')
    disp(i)
    disp('leading train position')
    disp(x0_l)
    disp('optimal control input')
    disp(uOpt_l(1))
    % next MPC step   
    i = i + 1 ;
    toc
    
    end
    
    
feas{1} = feas_l ;
feas{2} = feas_f ;

xOpt{1} = xOpt_l ;
xOpt{2} = xOpt_f ;

uOpt{1} = uOpt_l ;
uOpt{2} = uOpt_f ;

% pred error not implemented yet
predErr = [] ;


end