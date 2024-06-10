clear
clc

% Define the known parameters
global y_b H x D g d;
x = input("Enter the distance from the thrower to the building: ");
H = input("Enter the height of the building: ");
y_b = 3;    % Height of the basket in meters
d = 6;      % Distance from building to basket in meters
D = x + d;  % Total horizontal distance from thrower to basket
g = 9.81;   % Acceleration due to gravity in m/s^2

% Define the system of equations to solve
function F = equations(vars)
    global y_b H x D g;
    theta = vars(1);
    v0 = vars(2);
    % Safety margin to ensure the ball clears the building
    safety_margin = 1e-6; % 0.5 meters above the building height

    % Equation for the vertical position at the basket
    eq1 = y_b - (D * tan(theta) - (1/2) * g * (D / (v0 * cos(theta)))^2);
    
    % Equation for the height at the building
    eq2 = H + safety_margin - (x * tan(theta) - (1/2) * g * (x / (v0 * cos(theta)))^2);
    
    % Return the system of equations
    F = [eq1; eq2];
end

% Initial guesses for theta and v0
initial_guess = [0.000001, 0.000001];  % Initial guess: 0.000001 degrees and 0.000001 m/s

% Solve the system of equations using fsolve
options = optimoptions('fsolve', 'MaxFunEvals', 1e+50, 'MaxIterations', 1e+50);
solution = fsolve(@equations, initial_guess, options);

% Extract the solutions
alpha = solution(1);
theta_solution_deg = solution(1)*180/pi();
V = solution(2);
t_flight = D / (V * cos(alpha));


t = 0:1e-7:t_flight; % Time array
xe = V * cos(alpha) * t; % X - Axis
y = V * sin(alpha) * t - 0.5 * g * t.^2; % Y - Axis

plot(xe,y)
grid on;
grid minor;


% Display the results
fprintf('Initial angle (theta): %.2f degrees\n', theta_solution_deg);
fprintf('Initial angle (rad): %.2f radians\n', alpha)
fprintf('Initial velocity (v0): %.2f m/s\n', V);
fprintf('Time of flight: %.2f s', t_flight)