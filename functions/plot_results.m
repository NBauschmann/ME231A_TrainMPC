function plot_results(feas, xOpt_l, uOpt_l, xOpt_f, uOpt_f, predErr, x_pred_l, x_pred_f, p_sampled, DPspeed_, maxspeed_,fig)
%% function plots the results

%% Inputs 
% all the outputs from MPC.m
% feas, xOpt, uOpt, predErr, x_pred_l, x_pred_f
% p_sampled     sample points of position
% DPspeed_      fcn handle
% maxspeed_     fcn handle
% fig           indicates which figuere is used, set fig = [] for
%               sequential numeration
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% More plots should be added
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% enables choosing the figure number
if isempty(fig)
    figure()
else
    figure(fig)
end


plot(xOpt_l(1,:),xOpt_l(2,:),'ro')
hold on
if ~isempty(xOpt_f) % following train is also simulated
    plot(xOpt_f(1,:),xOpt_f(2,:),'bo')
    hold on
end
plot(x_pred_l{end}(1,:),x_pred_l{end}(2,:),'go')
hold on
if ~isempty(x_pred_f) % following train is also simulated
    plot(x_pred_f{end}(1,:),x_pred_f{end}(2,:),'go')
    hold on
end
plot(p_sampled,DPspeed_(p_sampled),'b')
hold on
plot(p_sampled,maxspeed_(p_sampled),'r')
grid on
xlabel('position [m]')
ylabel('velocity [m/s]')
title('Closed Loop Trajectory')
if ~isempty(xOpt_f)
    legend('Leading Train','Following Train','Last Prediction Leading Train','Last Prediction Following Train','DPspeed','maxspeed')
else 
    legend('Leading Train','Last Prediction Leading Train','DPspeed','maxspeed')
end


end