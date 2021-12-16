function [sigma] = compass_variance(ground_truth,compass)
%% Compute the variance of the signal

% Remove INVALID values
ind = compass ~= 1000;
compass = compass(ind);
ground_truth = ground_truth(ind);

% TODO: Deduce ground_truth from compass 
compass_error = compass-ground_truth;

% TODO: Remove excessive errors (> pi) from compass_error due to angle wind-up 
compass_error = compass_error(compass_error < pi);

% TODO: Compute the standard deviation of compass_error  
sigma = std(compass_error);

% Plot the data 
figure('Name','Compass');
%
subplot(2,1,1)
hold on;

plot(compass_error)
%plot(ground_truth(ind),'--','Color',[0.2,0.2,0.2])

title('Compass measurement errors')
xlabel('Samples')
ylabel('error [rad]')
legend({'compass'});%,'ground truth'})

%

% plot
subplot(2,1,2)
hold on;

histogram(compass_error)
yyaxis right
a = linspace(-10*sigma, 10*sigma, 100);
plot(a,normpdf(a,0,sigma),'--','Color','b')

yyaxis right
set(gca, 'YColor','b')

title('Compass noise')
xlabel('error [rad]')
yyaxis left
ylabel('samples count')
yyaxis right
ylabel('probability density')
legend({'compass error','distribution'})

end