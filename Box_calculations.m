%% sideway obstacle 
close all

% 376 565


figure()
plot(X(378:453),Y(378:453), "HandleVisibility","off")
hold on
y_ground_truth =linspace(0,10,1000);
x_ground_truth = 20*ones(1000,1);
plot(x_ground_truth,y_ground_truth)
hold on 
mld = fitlm(X(378:453),Y(378:453))
plot(mld)
axis([19 21 0 10])
grid on; grid minor;
title("Detected box centre against ground thruth")
legend("Ground thruth","Centre box position","Estimated trajectory")
xlabel("Distance on the x-axis [m]"); ylabel("Distance on the y-axis [m]");


figure()
plot(Y(378:453),width(378:453))
hold on
plot(Y(378:453),length(378:453))
legend("width","length")

figure()
plot(Y(378:453),sqrt(velX(378:453).^2+VelY(378:453).^2))
hold on
plot(Y(378:453),velX(378:453))
plot(Y(378:453),VelY(378:453))
legend("Velocity", "Velocity X", "Velocity Y")
xlabel("Distance on the y-axis [m]")
ylabel("Velocity [m/s]")
title("Velocity of the obstacle")
grid on; grid minor;
axis([2 10 0 1.4])
mean_velocity = mean(sqrt(velX(388:453).^2+VelY(388:453).^2))
mean_velocity_x = mean(velX(388:453))
mean_velocity_y = mean(VelY(388:453))
figure()
plot(X(378:453)-(width(378:453).*sin(orienZ(378:453))+length(378:453).*cos(orienZ(378:453)))/2,Y(378:453),'-x')
%axis([18.5 19.5 1 10])
%plot(X(378:453)-sqrt(length(378:453).^2+width(378:453).^2)/2,Y(378:453),'o')
title("Detected box corner")
xlabel("Distance on the x-axis [m]"); ylabel("Distance on the y-axis [m]");
grid on; grid minor