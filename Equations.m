%{
    given initial velocity, angle of launch, initial height, mass of the projectile and acceleration write equations to find final position, coordinates at max height, max distance, and time of flight. 
%}

function [final_position, max_height_coordinates, max_distance, time_of_flight] = projectile_motion(initial_velocity, angle_of_launch, initial_height, acceleration)
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