close all
clear all

load 2021-09-12-17-42-15.mat

t_TargetVel = TargetVel_data(:,1) - TargetVel_data(1,1);
t_ActualVel = ActualVel_data(:,1) - TargetVel_data(1,1);

figure(1)
plot(t_TargetVel,TargetVel_data(:,2));
hold on
plot(t_ActualVel,ActualVel_data(:,2));
title('t - RPM')
legend('TargetVel','ActualVel')

figure(2)
histogram(diff(t_TargetVel));
title('Discrete Time Histogram of TargetVelocity')

figure(3)
histogram(diff(t_ActualVel));
title('Discrete Time Histogram of ActualVelocity')

figure(4)
plot(diff(t_TargetVel))

figure(5)
plot(diff(t_ActualVel))