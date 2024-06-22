% MATLAB script to find the initial velocity, initial angle, and time of flight

% Given parameters
h0 = 2; % initial height from which the ball is thrown (in meters)
H = 4; % height of the building (in meters)
x = 5; % distance from the thrower to the building (in meters)
total_distance = x + 6; % total horizontal distance to the basket (in meters)
g = 9.81; % acceleration due to gravity (in m/s^2)

% Discretize search space
v0_range = 5:0.1:20; % range of initial velocities (m/s)
theta_range = deg2rad(10):deg2rad(0.5):deg2rad(80); % range of angles (radians)

% Initialize variables to store the best solution
best_error = inf;
best_v0 = 0;
best_theta = 0;

% Iterate over all combinations of v0 and theta
for v0 = v0_ran
    for theta = theta_range
        % Calculate time to reach the basket
        tf = total_distance / (v0 * cos(theta));
        
        % Calculate vertical position at the basket
        y_basket = h0 + v0 * sin(theta) * tf - 0.5 * g * tf^2;
        
        % Calculate time to reach the building
        tb = x / (v0 * cos(theta));
        
        % Calculate vertical position at the building
        y_building = h0 + v0 * sin(theta) * tb - 0.5 * g * tb^2;
        
        % Calculate the error terms
        error_basket = abs(y_basket - 3);
        error_building = max(0, H - y_building); % penalty if it doesn't clear the building
        
        % Total error
        total_error = error_basket + error_building;
        
        % Update best solution if this one is better
        if total_error < best_error
            best_error = total_error;
            best_v0 = v0;
            best_theta = theta;
        end
    end
end

% Calculate the time of flight for the best solution
tf_sol = total_distance / (best_v0 * cos(best_theta));

% Output the results
fprintf('Initial velocity (v0): %.2f m/s\n', best_v0);
fprintf('Initial angle (theta): %.2f degrees\n', rad2deg(best_theta));
fprintf('Time of flight (tf): %.2f seconds\n', tf_sol);

% Check the building clearance for the best solution
tb_sol = x / (best_v0 * cos(best_theta));
y_building = h0 + best_v0 * sin(best_theta) * tb_sol - 0.5 * g * tb_sol^2;

if y_building >= H
    disp('The ball clears the building.');
else
    disp('The ball does not clear the building.');
end
