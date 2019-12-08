function [xbar, ubar] = a_priori_estimation(xt,uOpt,param,MODEL,...
    slope_,radius_,limspeed_,maxspeed_) 
%% computes the a priori estimation based on the last solution of the CFTOP
% xbar(1)|t = xt
% xbar(k+1)|t = f(xbar(k)|t,ubar(k)|t) for k=1...Np-1
% ubar(k)|t = u(k)|t-1 for k=1...Np-2
% ubar(Np)|t = u(Np-1)|t-1
%% Inputs
% xt    current state
% uOpt  optimal control sequence from last CFTOC solution
%% Output
% xbar  next Np estimated states
% ubar  control sequence used for estimation
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
midterm = 1 ;
paper = 0 ;

Np = param.Np ;

ubar = [uOpt(:,2:end), uOpt(:,end)] ;

xbar = zeros(size(xt,1),Np+1) ;
xbar(:,1) = xt ;
for i = 1:Np
    if MODEL == midterm
        xbar(:,i+1) = train_dynamics_midterm(xbar(:,i), ubar(:,i), param,...
            slope_,radius_,limspeed_,maxspeed_) ;
    elseif MODEL == paper
        xbar(:,i+1) = train_dynamics(xbar(:,i), ubar(:,i), param,...
            slope_,radius_,limspeed_,maxspeed_) ;
    else
        error('MODEL is not assigned correctly')
    end
end

end