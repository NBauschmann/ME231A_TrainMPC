function [vlim]= limspeed(s,p_sampled,vOpt_DP)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% function that calculates the limiting speed that has been 
% found in the precomputation via Dynamic Programming
%% Inputs
% s         current position on track
% vOpt_DP   vector containing the DP solution
%% Outputs
% vlim      limiting speed at position p
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% use linear interpolation for velocity
vel = @(v) interp1(p_sampled,vOpt_DP,v,'linear');
vlim = vel(s);


end

