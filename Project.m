%{ 
The following suggested project can be used
1. Projectile; an object with a mass “M” was launched at origin with a velocity “V” and angle “α”.
2. The origin is located at a distance “D” of a building
3. The building has a height “H”
4. The object must land in a basketball ring located behind the building with the following
details:
- 3 meters above the ground
- 6 meters away from the building
5. The project works should contain the following as a minimum:
- Analyze the object movements using the dynamics equations
- Complete MATLAB programming to analyze and plot the object movement
- Have the ability to check the location of the object at any required moment
- Have the ability to check the maximum height the object can reach
- Create a Matlab application where the input is D and H, the output will be the velocity “V” and angle “α”.
- The program should have graphing options with and without GUI
- A report must be submitted including the codes
- Include the operating conditions of the code based on the input data (D & H)

%}

% Input values
D = input('Enter the distance from the building (D): ');
H = input('Enter the height of the building (H): ');

% Constants
g = 9.81; % Acceleration due to gravity (m/s^2)
basketball_ring_height = 3; % Height of the basketball ring (m)
basketball_ring_distance = 6; % Distance of the basketball ring from the building (m)

% Calculate the velocity and angle
V = sqrt((D^2 * g) / (D * tan(asin((H - basketball_ring_height) / D)) - H + basketball_ring_height));
alpha = atan((H - basketball_ring_height) / D);

% Display the results
disp(['Velocity (V): ' num2str(V) ' m/s']);
disp(['Angle (alpha): ' num2str(rad2deg(alpha)) ' degrees']);

% Plot the object's movement
t = linspace(0, 2 * V * sin(alpha) / g, 100);
x = V * cos(alpha) * t;
y = V * sin(alpha) * t - 0.5 * g * t.^2;

figure;
plot(x, y);
xlabel('Horizontal Distance (m)');
ylabel('Vertical Distance (m)');
title('Projectile Motion');
grid on;

% Check the location of the object at any required moment
t_check = input('Enter the time to check the location of the object: ');
x_check = V * cos(alpha) * t_check;
y_check = V * sin(alpha) * t_check - 0.5 * g * t_check^2;
disp(['At time ' num2str(t_check) ' s, the object is at position (' num2str(x_check) ', ' num2str(y_check) ')']);

% Calculate the maximum height
t_max_height = V * sin(alpha) / g;
y_max_height = V * sin(alpha) * t_max_height - 0.5 * g * t_max_height^2;
disp(['The maximum height the object can reach is ' num2str(y_max_height) ' m']);

%test Ayman%