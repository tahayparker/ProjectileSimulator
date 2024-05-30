%{
    a person is throwing a basketball from some x meters away. the basketball has to go over a building of height h, and land in the basket which is at a distance of 6m away on the OTHER side of the building. the basket is at a height of 3m from ground.
    write dynamics equations to find the following variables with the following inputs
    inputs:
        D - Distance from person to building (on the first side of the building)
        H - Height of the building 
    outputs:
        V - Initial velocity of the ball
        alpha - angle at which the ball was launched
        t - time of flight
    constraints:

        1. the distance D cannot be negative
        2. the distance D cannot be zero
        3. the height H cannot be negative
        4. the height H cannot be zero
        5. the distance D has to be greater than 6m
        6. the height H has to be greater than 3m
        
    code it in such a way that if the person is too close to the building, i.e. the D is very less, then it will inform the user that it is not possible to shoot at such an angle

%}

%}

%{
    let the initial velocity of the ball be V and the angle at which the ball is launched be alpha
    let the time of flight be t
    let the acceleration due to gravity be g = 9.81 m/s^2
    let the height of the building be H
    let the distance from the person to the building be D
    let the distance from the building to the basket be 6m
    let the height of the basket be 3m
%}

%{
    the dynamics equations are as follows:
    1. the horizontal distance covered by the ball is given by:
        D = V*cos(alpha)*t
    2. the vertical distance covered by the ball is given by:
        H = V*sin(alpha)*t - 0.5*g*t^2
    3. the time of flight is given by:
        t = 2*V*sin(alpha)/g
    4. the horizontal distance covered by the ball is given by:
        6 = V*cos(alpha)*t
    5. the vertical distance covered by the ball is given by:
        3 = V*sin(alpha)*t - 0.5*g*t^2

%}

%{
    the above equations can be solved to find the values of V, alpha and t
    the equations can be solved using the following steps:
    1. find the value of t using equation 3
    2. substitute the value of t in equations 1 and 2 to find the values of V and alpha

%}

%{
    the code to solve the above equations is as follows:

%}

function [V, alpha, t] = eqs(D, H)
    % check if the inputs are valid
    if D <= 0
        error('Distance D cannot be negative or zero');
    end
    if H <= 0
        error('Height H cannot be negative or zero');
    end
    if D <= 6
        error('Distance D has to be greater than 6m');
    end
    if H <= 3
        error('Height H has to be greater than 3m');
    end
    
    % define the acceleration due to gravity
    g = 9.81;
    
    % calculate the time of flight
    t = 2*sqrt((H-3)/g);
    
    % calculate the initial velocity
    V = 6/(t*cos(atan((H-3)/6)));
    
    % calculate the angle of launch
    alpha = atan((H-3)/6);
    
    % check if the angle is possible
    if alpha > pi()/2
        error('It is not possible to shoot at such an angle');
    end
end
