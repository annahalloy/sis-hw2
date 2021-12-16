function [error, metric] = metric(error_x,error_y) 
    % Input: 
    %   error_x: (1D array) absolute x error 
    %   error_y: (1D array) absolute y error 
    % Ouput: 
    %   error: (1D array) element wise norm of error_x and error_y
    %   metric:(scalar) metric evaluation of the error  
    error = square(error_x.^2+error_y.^2); % or simply error_x + error_y ? 
    metric = error./(length(error)-1:length(error));
end