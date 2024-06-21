% Define the equation as a function handle
eqn = @(alpha) H - (D * tan(alpha) - (1/2) * (D^2 * (alphaf * sin(2 * alpha) - 6 * (1 - sin(alpha)^2))) / (alphaf^2 * cos(alpha)^2));

% Define the range of x to search for solutions and the step size
x_start = 0;
x_end = 2 * pi;
step_size = 0.000001;

% Initialize an array to store the solutions
solutions = [];

% Loop through the range of x values
for x = x_start:step_size:x_end
    % Evaluate the equation at the current and next points
    y1 = eqn(x);
    y2 = eqn(x + step_size);
    
    % Check for a sign change (root crossing)
    if y1 * y2 < 0
        % Use a finer resolution to find a more accurate root
        finer_step_size = step_size / 10;
        for finer_x = x:finer_step_size:(x + step_size)
            finer_y1 = eqn(finer_x);
            finer_y2 = eqn(finer_x + finer_step_size);
            if finer_y1 * finer_y2 < 0
                root = (finer_x + finer_x + finer_step_size) / 2;
                solutions = [solutions; root];
                break;
            end
        end
    end
end

% Display the solutions
disp('Solutions for x:');
disp(solutions);
