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
%     - fmincon runs for a very long time 
%          -> find run time improvements
%     - write plotting functions, etc. 
%     - implement initial DP for paper model
% For Midterm Model: 
%          - MPC is running and tested, takes a long time
% ....
% IDEAS THAT COULD BE EXPLORED:
%       - execute more than the first input of the COFTC problem
%       - LMPC: not sure if applicable here, we construct safe set
%         by setting the terminal constraint (velocity at last step)
%         according to the precomputed limspeed_ from DP precomputation
%       - use DP instead of CFOTC ?
%       - use two following trains
%       - shorter prediction horizon
%       - moving block
%       - uncertainties in position, distance and line profile
%       - soft - constraints
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
    x0_l = [54;10;0] ;
    x0_f = [0;10;0] ;

elseif MODEL == midterm
    % initial condition
    x0_l = [54;10] ;
    x0_f = [0;10] ;

end

% [feas, xOpt, uOpt, predErr, x_pred_l, x_pred_f] = MPC(x0_l, x0_f,param,MODEL,...
%         slope_,radius_,limspeed_,maxspeed_,p_sampled)

[feas, xOpt, uOpt, predErr, x_pred_l, x_pred_f] = MPC_leadingTrain(x0_l, x0_f,param,MODEL,...
        slope_,radius_,limspeed_,maxspeed_,p_sampled)
    

%%

plot_results(feas, xOpt, uOpt, predErr, x_pred_l, x_pred_f, p_sampled, limspeed_)


