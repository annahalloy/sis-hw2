
data_name = '/Users/annahalloy/Documents/EPFL/BA5/SIS/sis-hw2/code_assignment/part1/webots/controllers/e-puck_supervisor/supervisor_logfile.txt';

A = importdata(data_name);


x = A(:,2);
z = A(:,3);

figure
plot(x,z)
xlabel('x', 'fontsize', 14)
ylabel('z', 'fontsize',14)
title('Trajectory of the robot', 'fontsize', 18)
grid on

