function [sigma] = distance_sensor_variance(ds_id, ds_range)
%% Compute the variance of the distance sensors 

% Find the sensor that returns the shortest range 
[~,ind] = min(ds_range);
sensor_id = ds_id(ind); 

% Select values for the corresponding sensor
index = ds_id == sensor_id; 
distances = ds_range(index);

% TODO: Remove invalid data (=-1) from the 'distances' array  
distances = distances(distances ~= -1);

% TODO: Compute the standard deviation of distances
sigma = std(distances);


figure('Name','DistanceSensor');
%
subplot(2,1,1)
hold on;

plot(distances)

title('Distance sensor measurements')
xlabel('Samples')
ylabel('measurements [m]')
legend({'distance sensor'})
%
subplot(2,1,2)

hold on;

histogram(distances - mean(distances))
a = linspace(-10*sigma, 10*sigma, 100);
yyaxis right
plot(a,normpdf(a,0,sigma),'--','Color','b')

yyaxis right
set(gca, 'YColor','b')

title('Distance sensor noise')
xlabel('error [m]')
yyaxis left
ylabel('samples count')
yyaxis right
ylabel('probability density')
legend({'ds error','distribution'})

end