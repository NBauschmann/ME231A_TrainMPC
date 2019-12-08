%% main Virtual Coupling of Trains 
% Final Project ME 231A
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Authors
% Tom Heitmann, Nathalie Bauschmann, Pierre-Louis Blossier, Johannes
% Nilssen, Antonia Bronars
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Midterm and Paper use different models
%
% TO DO:
% Paper Model: - CFTOC init state violates boundary
%              - fmincon runs for a very long time and 
%                converges to an nonfeasible point
%                -> use DP instead???
% For Midterm Model: - CFTOC has to be implemented
%          - MPC
% ....
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear all
close all
clc
%% set up path
addpath('data','functions')

%% General Setup
midterm = 1 ;
paper = 0 ;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% choose midterm or paper model
MODEL = midterm ;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% parameter setup
% loads profile, parameters, veloc
load train_data_midterm

clear param
param = setup_parameters(MODEL); 

%% precomputation of optimal speed profile via Dynamic programming

load DP_results

%% define function interpolation for radius, slope, maxspeed, limspeed
slope_ = @(v) 0.001 * interp1(profile(:,1),profile(:,2),v,'previous');
radius_ = @(v) interp1(profile(:,1),profile(:,3),v,'previous');
limspeed_ = @(v) interp1(p_sampled,vOpt_DP,v,'previous');
maxspeed_ = @(v) 1/3.6 * interp1(veloc(:,1),veloc(:,2),v,'previous');

% this could be brought to setup_param

%% perform MPC
if MODEL == paper
    % initial state
    x0_l = [60;10;0] ;
    x0_f = [0;10;0] ;

elseif MODEL == midterm
    % initial condition
    x0_l = [80;10] ;
    x0_f = [0;10] ;

end

[feas, xOpt, uOpt, predErr] = MPC(x0_l, x0_f,param,MODEL,...
        slope_,radius_,limspeed_,maxspeed_,p_sampled);


%%
% feas_l = [] ;
% feas_f = [] ;
% 
% xOpt_l = x0_l;
% xOpt_f = x0_f;
% 
% uOpt_l = [] ;
% uOpt_f = [] ;
% 
% x_pred_l = [] ;
% x_pred_f = [] ;
% 
% % predErr_l = zeros(nx,1);
% % predErr_f = zeros(nx,1);
% 
% % initialisation for apriori estimation before first step
% uOpt_l_t = 10e+05*ones(1,param.Np) ;
% uOpt_f_t = 10e+05*ones(1,param.Np) ;
% 
% % indicator of MPC iteration
% i = 1 ;
% 
% 
% % apriori estimation
% [xbar_l, ubar_l] = a_priori_estimation(x0_l, uOpt_l_t, param, MODEL, ...
%     slope_, radius_, limspeed_, maxspeed_);
% [xbar_f, ubar_f] = a_priori_estimation(x0_f, uOpt_f_t, param, MODEL, ...
%     slope_, radius_, limspeed_, maxspeed_);
% 
% %% execute finite time horizon optimization
% [feas_l_t, xOpt_l_t, uOpt_l_t, JOpt_l_t] = cftoc_leadingTrain(x0_l, ... 
%    xbar_l, ubar_l, MODEL, param, slope_,radius_,limspeed_,maxspeed_) ;
% 

%%
% if feas_l_t ~= 1
%     xOpt_l = [];
%     xOpt_f = [];
%     uOpt_l = [];
%     uOpt_f = [];
% %         predErr_l = [];
% %         predErr_f = [];
%     feas_l = [feas_l, false];
% 
% 
% else
%    feas_l = [feas_f, true];
%    x_pred_l{i} = xOpt_l_t ;
% 
%     % execute finite time horizon optimization for following train
%     [feas_f_t, xOpt_f_t, uOpt_f_t, JOpt_f_t] = cftoc_followingTrain(x0_f, ... 
%         xbar_f, ubar_f, xOpt_l_t(:,1), uOpt_l_t, MODEL, param, slope_,radius_,limspeed_,maxspeed_) ;
% 
%    if feas_f_t ~= 1
%         xOpt_l = [];
%         xOpt_f = [];
%         uOpt_l = [];
%         uOpt_f = [];
% %             predErr_l = [];
% %             predErr_f = [];
%         feas_f = [feas_f, false];
% 
%    else
%         feas_f = [feas_f, true];
%         xOpt_l = [xOpt_l, xOpt_l_t(:,2)];
%         xOpt_f = [xOpt_f, xOpt_f_t(:,2)];
%         uOpt_l = [uOpt_l, uOpt_l_t(1,1)];
%         uOpt_f = [uOpt_f, uOpt_f_t(1,1)];
%         x0_l = xOpt_l_t(:,2) ;
%         x0_f = xOpt_f_t(:,2) ;
%         x_pred_f{i} = xOpt_f_t ;
% 
%    end
% 
% 
% end



