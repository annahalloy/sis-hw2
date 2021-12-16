function [sigma] = odometry_variance(x,x_hat,var,unit)

% TODO: Compute the error between estimation x_hat and ground truth x
error = x_hat - x;

if strcmp(var, 'heading')
    index = abs(error) < pi;
    error = error(index);
end

% Remove possible nan values
ind = ~isnan(error);
error = error(ind);

% TODO: Compute the standard deviation of the error 
sigma = std(error);

% TODO: Remove the mean from the error (for histogram)
error_centered = error - mean(error);

% Plot the data 
figure('Name','Noise odometry');
%
subplot(2,1,1)
hold on;
grid on; 

plot(error)

title("Odometry error in " + var)
xlabel('Samples')
ylabel("error "+unit)

%

% plot
subplot(2,1,2)
hold on;

histogram(error_centered)
yyaxis right
a = linspace(-10*sigma, 10*sigma, 100);
plot(a,normpdf(a,0,sigma),'--','Color','b')

yyaxis right
set(gca, 'YColor','b')

title("Odometry noise in " + var)
xlabel("error "+unit)
yyaxis left
ylabel('samples count')
yyaxis right
ylabel('probability density')
legend({var+" error",'distribution'})

end