% model parameters
% as given in "A model predictive control approach for virtual coupling in 
% railways"


param.g = 9.81 ;
% mass
param.M = 1.608e+5 ;
% length of the train
param.L = 68 ;
% rolling & bearing resistence
param.A = 9155 ;
% air input coefficient
param.B = 633.6 ;
% aerodynamic coeff
param.C = 46.84 ;
% inertial lag of longitudinal dynamics
param.tau = 0.25;
% diving power
param.Pdr = 1.20e+06 ;
% breaking power
param.Pbr = 2.20e+06 ;
% driving acceleration
param.adr = 0.9 ;
% breaking acceleration
param.abr = 0.92 ;
% 
param.jmax = 0.98 ;
% 
param.a_l = 1.25 ;
% 
param.a_f = 0.75 ; % 0.5 ... 1.0

% MPC parameters

% time step
param.delta_t = 0.1 ;
% desired distance 
param.d_des = 10.0 ;
% minimal distance
param.d_min = 4.0 ;
% finite time horizon
param.Np = 30 ;
% weights in cost functions
param.Kv_l = 100 ;
param.Kj_l = 0.5 ;
param.Kd = blkdiag(100,25,0);
param.Kv_f = 0.5 ;
param.Kj_f = 0.5 ;
param.Kv_DP = 1.0 ;
param.Kl_DP = 1.0e+05 ;
