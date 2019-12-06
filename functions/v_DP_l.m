function [v_DP] = v_DP_l(s,p_sampled,vOpt_DP)
% function that outputs the speed that was precomputed by   
% the dynamic programming approach
% global p_sampled 
% global vOpt
len = length(p_sampled);

for i= 1:len-1
    if s > p_sampled(i) && s <= p_sampled(i+1)
        v_DP = vOpt_DP(i);
    end
end
    if s >= p_sampled(len)
        v_DP = vOpt_DP(len);
    end
end