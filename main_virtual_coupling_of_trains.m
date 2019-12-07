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


%% check CFTOC solution
if MODEL == paper
    % initial state
    x0 = [0;0;0] ;

    % apriori estimation
    uOpt0 = 00*ones(1,param.Np);
    xt = x0;
    [xbar, ubar] = a_priori_estimation(xt,uOpt0,param,MODEL); 
    %

    [feas_l, xOpt_l, uOpt_l, JOpt_l] = cftoc_leadingTrain(x0, ... 
       xbar, ubar, MODEL, param, p_sampled, vOpt_DP)
elseif MODEL == midterm
  x0 = [0;0] ;

    % apriori estimation
    uOpt0 = 00*ones(1,param.Np);
    xt = x0;
    [xbar, ubar] = a_priori_estimation(xt,uOpt0,param,MODEL); 
    

    [feas_l, xOpt_l, uOpt_l, JOpt_l] = cftoc_leadingTrain(x0, ... 
       xbar, ubar, MODEL, param, p_sampled, vOpt_DP)
end



