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
    x0_l = [60;0.1;0] ;
    x0_f = [0;0.1;0] ;

elseif MODEL == midterm
    % initial condition
    x0_l = [60;0.1] ;
    x0_f = [0;0.1] ;

end

[feas, xOpt, uOpt, predErr] = MPC(x0_l, x0_f,param,MODEL,...
        slope_,radius_,limspeed_,maxspeed_,p_sampled);



