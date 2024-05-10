
t= linspace(0, 2 * V * sin(alpha) / g, 100);
 x = V * cos(alpha) * t;
    y = V * sin(alpha) * t - 0.5 * g * t.^2;

plot(x,y)

hold on

hold off