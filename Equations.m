%{
    given initial velocity, angle of launch, initial height, mass of the projectile and acceleration write equations to find final position, coordinates at max height, max distance, and time of flight. 
%}

function [final_position, max_height_coordinates, max_distance, time_of_flight] = projectile_motion(initial_velocity, angle_of_launch, initial_height, mass, acceleration, air_resistance)
    % Convert angle from degrees to radians
    angle_of_launch = deg2rad(angle_of_launch);

    % Initial conditions
    x = 0; % initial horizontal position
    y = initial_height; % initial vertical position
    vx = initial_velocity * cos(angle_of_launch); % initial horizontal velocity
    vy = initial_velocity * sin(angle_of_launch); % initial vertical velocity

    % Time step
    dt = 0.01; % seconds

    % Initialize arrays to store the trajectory
    X = [];
    Y = [];

    % While the projectile is still in the air
    while y >= 0
        % Append current position to the trajectory
        X = [X x];
        Y = [Y y];

        % Update velocity and position
        vx = vx - air_resistance/mass * vx * dt;
        vy = vy - (acceleration + air_resistance/mass * vy) * dt;
        x = x + vx * dt;
        y = y + vy * dt;
    end

    % Output
    final_position = [X(end), Y(end)];
    % Time at which the projectile reaches max height 
    [~, index] = max(Y);
    time_at_max_height = index * dt;
    max_height_coordinates = [X(Y == max(Y)), max(Y)];
    max_distance = X(end);
    time_of_flight = length(X) * dt;
end

% OLD CODE

%{

function [final_position, max_height_coordinates, max_distance, time_of_flight] = projectile_motion(initial_velocity, angle_of_launch, initial_height, mass, acceleration)
    % Convert angle from degrees to radians
    angle_of_launch = deg2rad(angle_of_launch);

    % Time of flight
    time_of_flight = (2 * initial_velocity * sin(angle_of_launch)) / acceleration;

    % Max distance
    max_distance = initial_velocity * cos(angle_of_launch) * time_of_flight;

    % Coordinates at max height
    time_at_max_height = initial_velocity * sin(angle_of_launch) / acceleration;
    max_height = initial_height + initial_velocity * sin(angle_of_launch) * time_at_max_height - 0.5 * acceleration * time_at_max_height^2;
    max_height_coordinates = [initial_velocity * cos(angle_of_launch) * time_at_max_height, max_height];

    % Final position
    final_position = [max_distance, initial_height - 0.5 * acceleration * time_of_flight^2];
end

%}