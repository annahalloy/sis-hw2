%% Part A: Plot the ground truth and odometry 

[~,~,~,data] = read_log();

figure('Name','Odometry', 'Position',[1000 200 500 800]); % x y width height

% Plot trajectory
subplot(2,1,1)
hold on;
xlim([-0.55 0.55])
ylim([-0.55 0.55])
pbaspect([1 1 1])

scatter(0,0,'black')

plot(data.gt_x,data.gt_y,'blue')
plot(data.odo_enc_x,data.odo_enc_y,'red')

scatter(data.gt_x(end-1),data.gt_y(end-1),'blue','x','LineWidth',1.5)
scatter(data.odo_enc_x(end-1),data.odo_enc_y(end-1),'red','x','LineWidth',1.5)

plot([-1,1,1,-1,-1]*0.5,[-1,-1,1,1,-1]*0.5,'-','Color',[0.2,0.2,0.2])

title('Trajectory')
xlabel('x [m]')
ylabel('y [m]')
legend({'start','ground truth','odometry'})

% Plot heading 
subplot(2,1,2)
hold on; 

plot(data.time,data.gt_heading,'blue')
plot(data.time,data.odo_enc_heading,'red')

plot([data.time(1),data.time(end)],[pi,pi],'--','Color',[0.2,0.2,0.2])
plot([data.time(1),data.time(end)],[-pi,-pi],'--','Color',[0.2,0.2,0.2])

title('Heading')
xlabel('t [s]')
ylabel('heading [rad]')
legend({'ground truth','odometry'})

%% Part B: Compute metric odometry 

[~, T_SIM, T, data] = read_log();

% Metric coomputation
[error_x,metric_x] = metric_scalar(data.gt_x(1:end-1), data.odo_enc_x(1:end-1)); 
[error_y,metric_y] = metric_scalar(data.gt_y(1:end-1), data.odo_enc_y(1:end-1)); 
[error_xy, metric_xy] = metric(error_x,error_y);

disp(['Metric odometry x: ',num2str(metric_x)])
disp(['Metric odometry y: ',num2str(metric_y)])
disp(['Metric odometry combined: ',num2str(metric_xy)])

figure('Name','Trajectory error odometry')

hold on; 
grid on; 

plot(T_SIM(1:end-1)*T, error_x,'--','Color','b')
plot(T_SIM(1:end-1)*T, error_y,'-','Color','b')
plot(T_SIM(1:end-1)*T, error_xy,'Color','r','LineWidth',2)

title("Pose error odometry, metric: "+num2str(metric_xy))
xlabel('t [s]')
ylabel('error [m]')
legend({'x','y','combined'})

%% Part C: Plot the compass data

[~,~,~,data] = read_log();

figure('Name','Compass');

hold on;

% ignore invalid data
ind = data.compass < 10; 

plot(data.time,data.gt_heading,'blue')
scatter(data.time(ind),data.compass(ind),'red','x')

plot([data.time(1),data.time(end)],[pi,pi],'--','Color',[0.2,0.2,0.2])
plot([data.time(1),data.time(end)],[-pi,-pi],'--','Color',[0.2,0.2,0.2])

title('Heading')
xlabel('t [s]')
ylabel('heading [rad]')
legend({'ground truth','compass'})

%% Part D: Compute the compass std

[~,~,~,data] = read_log();

[sigma] = compass_variance(data.gt_heading,data.compass);

disp(['Standard deviation of compass angle: ',num2str(sigma)])

%% Part E: Compute the distance sensors std

[~,~,~,data] = read_log();

[sigma] = distance_sensor_variance(data.ds_id, data.ds_range);

disp(['Standard deviation of distance sensor: ',num2str(sigma)])

%% Part F: Compute the odometry std 

[~,~,~,data] = read_log();

[sigma_x] = odometry_variance(data.gt_x,data.odo_enc_x,'x','[m]');
[sigma_y] = odometry_variance(data.gt_y,data.odo_enc_y,'y','[m]');
[sigma_heading] = odometry_variance(data.gt_heading,data.odo_enc_heading,'heading','[rad]');

disp(['Standard deviation of odometry (x,y,heading): ', ...
    num2str(sigma_x),', ',num2str(sigma_y),', ',num2str(sigma_heading)])

%% Part G: Plot the wall detections 
% Note: better to use ground truth as reference 

[~,~,~,data] = read_log();

figure('Name','Trajectory and wall detection'); 

hold on;
xlim([-0.55 0.55])
ylim([-0.55 0.55])
pbaspect([1 1 1])

scatter(0,0,'black')

plot(data.gt_x,data.gt_y,'blue')
%plot(data.odo_enc_x,data.odo_enc_y,'red')
ind = data.wall_x < 100; 
scatter(data.wall_x(ind),data.wall_y(ind),'cyan')

plot([-1,1,1,-1,-1]*0.5,[-1,-1,1,1,-1]*0.5,'-','Color',[0.2,0.2,0.2])

scatter(data.gt_x(end-1),data.gt_y(end-1),'blue','x','LineWidth',1.5)

title('Wall detections')
xlabel('x [m]')
ylabel('y [m]')
legend({'start','ground truth','wall detections'}) % 'odometry',

%% Part H: Plot the Kalman filter trajectory

[~,~,~,data] = read_log();

figure('Name','Trajectory', 'Position',[1000 200 800 1000]); % x y width height

% Plot trajectory
subplot(2,1,1)
hold on;
xlim([-0.55 0.55])
ylim([-0.55 0.55])
pbaspect([1 1 1])

scatter(0,0,'black')

plot(data.gt_x,data.gt_y,'blue')
plot(data.odo_enc_x,data.odo_enc_y,'red')
plot(data.kalman_x,data.kalman_y,'green')
ind = data.wall_x < 10; 
scatter(data.wall_x(ind),data.wall_y(ind),'cyan')

scatter(data.gt_x(end-1),data.gt_y(end-1),'blue','x','LineWidth',1.5)
scatter(data.odo_enc_x(end-1),data.odo_enc_y(end-1),'red','x','LineWidth',1.5)
scatter(data.kalman_x(end-1),data.kalman_y(end-1),'green','x','LineWidth',1.5)

plot([-1,1,1,-1,-1]*0.5,[-1,-1,1,1,-1]*0.5,'-','Color',[0.2,0.2,0.2])

title('Trajectory')
xlabel('x [m]')
ylabel('y [m]')
legend({'start','ground truth','odometry','kalman','wall detections'})

% Plot heading 
subplot(2,1,2)
hold on; 

plot(data.time,data.gt_heading,'blue')
plot(data.time,data.odo_enc_heading,'red')
plot(data.time,data.kalman_heading,'green')
ind = data.compass < 10; 
scatter(data.time(ind),data.compass(ind),'cyan','x')

plot([data.time(1),data.time(end)],[pi,pi],'--','Color',[0.2,0.2,0.2])
plot([data.time(1),data.time(end)],[-pi,-pi],'--','Color',[0.2,0.2,0.2])

title('Heading')
xlabel('t [s]')
ylabel('heading [rad]')
legend({'ground truth','odometry','kalman','compass'})


%% Part I: Compute metric kalman 

[N_SIM, T_SIM, T, data] = read_log();

% Metric coomputation
[error_x,metric_x] = metric_scalar(data.gt_x(1:end-1), data.kalman_x(1:end-1)); 
[error_y,metric_y] = metric_scalar(data.gt_y(1:end-1), data.kalman_y(1:end-1)); 
[error_xy, metric_xy] = metric(error_x,error_y);

disp(['Metric kalman x: ',num2str(metric_x)])
disp(['Metric kalman y: ',num2str(metric_y)])
disp(['Metric kalman combined: ',num2str(metric_xy)])

figure('Name','Trajectory error kalman')

hold on; 
grid on;

plot(T_SIM(1:end-1)*T, error_x,'--','Color','b')
plot(T_SIM(1:end-1)*T, error_y,'-','Color','b')
plot(T_SIM(1:end-1)*T, error_xy,'Color','r','LineWidth',2)

title("Pose error kalman, metric: "+num2str(metric_xy))
xlabel('t [s]')
ylabel('error [m]')
legend({'x','y','combined'})
