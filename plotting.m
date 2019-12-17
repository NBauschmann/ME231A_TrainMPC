
addpath('data','functions')

%%

close all
clear
clc

load DP_results.mat
load train_data_midterm.mat

figure;
plot(p_sampled(2:end), uOpt);

figure;
plot(p_sampled, vOpt_DP);
 
 

%%

close all
clear
%load('DP_results.mat')
load('train_data_midterm.mat')
load('both_worked.mat')
pause(0.1)
clc

xOpt{1}(:,1) = x0_l;
for i = 1:685
    xOpt{1}(:,i+1) = x_pred_l{i}(:,2);
    uOpt{1}(:,i) = u_pred_l{i}(:,2);
end

xOpt_l = xOpt{1};
xOpt_f = xOpt{2};
uOpt_l = uOpt{1};
uOpt_f = uOpt{2};

predErr; %TODO
fig = [];

%%
figure;
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
    legend('Leading Train','Following Train','Last Prediction Leading Train','Last Prediction Following Train','Optimal safe speed','Speed Limit')
else 
    legend('Leading Train','Last Prediction Leading Train','DPspeed','maxspeed')
end

%%

figure;
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
    legend('Leading Train','Following Train','Last Prediction Leading Train','Last Prediction Following Train','Optimal safe speed','Speed Limit')
else 
    legend('Leading Train','Last Prediction Leading Train','DPspeed','maxspeed')
end

ylim([5, 50])
xlim([0, 1250])


%%

figure;
plot(xOpt_l(1,:),xOpt_l(2,:),'g')
hold on

plot(xOpt_f(1,:),xOpt_f(2,:),'b')
hold on

plot(p_sampled,maxspeed_(p_sampled),'r')
grid on

xlabel('position [m]')
xlim([0, 1250])
ylabel('velocity [m/s]')
ylim([4, 50])

title('Closed Loop Trajectory')
legend('Leader','Follower','Speed limit')

%%

figure;
l = size(xOpt_f,2);
distance = xOpt_l(1,:) - xOpt_f(1,:);
plot(xOpt_l(1,:),distance)
hold on
safedist = (xOpt_l(2,:)).^2.*param.M./(2*param.Pbrake);
load('Moving_Block.mat')
l = size(XOpt{1}(2,:),2);
distance = XOpt{1}(1,:) - XOpt{2}(1,:);
plot(XOpt{1}(1,:),distance)
plot(xOpt_l(1,:), safedist);

legend('Virtual Coupling', 'Moving Block', 'Leader braking length')
title('Train distance')
ylabel('length [m]')
xlabel('Leader position [m]')

%%
M = length(x_pred_l);
N = length(x_pred_l{1}(2,:));
predErr_l = zeros(4,M-N+1);
predErr_f = zeros(4,M-N+1);

for i=1:M-N+1
    predErr_l(1,i) = norm(xOpt_l(1,i+1:i+N-1) - x_pred_l{i}(1,2:N));
    predErr_l(2,i) = norm(xOpt_l(2,i+1:i+N-1) - x_pred_l{i}(2,2:N));
    predErr_f(1,i) = norm(xOpt_f(1,i+1:i+N-1) - x_pred_f{i}(1,2:N));
    predErr_f(2,i) = norm(xOpt_f(2,i+1:i+N-1) - x_pred_f{i}(2,2:N));
end

figure;
subplot(2,1,1)
plot(xOpt_l(1,1:end-N),predErr_l(1,:))
title({'2-norm prediction error', 'leader'})
ylabel('x_1')
subplot(2,1,2)
plot(xOpt_l(1,1:end-N),predErr_l(2,:))
ylabel('x_2')
xlabel('step, k')

figure;
subplot(2,1,1)
plot(xOpt_f(1,1:end-N),predErr_f(1,:))
title({'2-norm prediction error', 'follower'})
ylabel('x_1')
subplot(2,1,2)
plot(xOpt_f(1,1:end-N),predErr_f(2,:))
ylabel('x_2')
xlabel('step, k')


