function [e] = cost_cec2020(x, fNo)
    % Increment the number of function evaluations
    global countFE;
    countFE = countFE + 1 ; % inc function evaluations

    % Define the global minimum values for each function number
    globalMins = [100, 1100, 700, 1900, 1700, 1600, 2100, 2200, 2400, 2500];
    
    % Check if the function number is within the valid range
    if fNo < 1 || fNo > length(globalMins)
        e = Inf;
        fprintf('Function number must be between 1 and %d', length(globalMins));
        return
    end
    
    % Get the global minimum for the given function number
    globalMin = globalMins(fNo);
    
    % Call the CEC2020/2021 function 
    % The solution vector x is transposed to suit the CEC evaluation function requirements
    f = cec20_func(x', fNo); % Standard CEC2020 Library

    % Calculate the error as the difference between the evaluated function value and the global minimum
    e = f - globalMin;
    
end
