function plot_results(feas, xOpt, uOpt, predErr, x_pred_l, x_pred_f, p_sampled, DPspeed_, maxspeed_)

%% plot closed loop trajectory
% figure()
% plot(xOpt{1}(1,:),xOpt{1}(2,:),'ro')
% hold on 
% plot(xOpt{2}(1,:),xOpt{2}(2,:),'bo')
% hold on
% plot(p_sampled,DPspeed_(p_sampled))
% grid on
% xlabel('position [m]')
% ylabel('velocity [m/s]')
% title('Closed Loop Trajectory')
% legend('Leading Train','Following Train','limspeed')


figure()
plot(xOpt{1}(1,:),xOpt{1}(2,:),'ro')
hold on 
plot(x_pred_l{end}(1,:),x_pred_l{end}(2,:),'bo')
hold on
plot(p_sampled,DPspeed_(p_sampled))
hold on
plot(p_sampled,maxspeed_(p_sampled))
grid on
xlabel('position [m]')
ylabel('velocity [m/s]')
title('Closed Loop Trajectory')
legend('Leading Train','last prediction','DPspeed','maxspeed')





end