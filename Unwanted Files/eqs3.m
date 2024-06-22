Distance = 8;
Height = 10;
theta = 0:0.01:89.99;

v1 = sqrt(((Distance+6)^2*9.8) ./((2 .* (cosd(theta)).^2).*((Distance+6).*tand(theta)-3)));

v2 = sqrt(((Distance^2)*9.8) ./((2.*(cosd(theta)).^2).* (Distance.*tand(theta)-Height)));

plot(theta,v1,'r',theta,v2,'b');

% Find the intersection points of the two graphs
[~,idx] = min(abs(v1-v2));
hold on
plot(theta(idx),v1(idx),'ko');
legend('v0 for basket','v0 for building','Intersection point');
xlabel('Theta (degrees)');
ylabel('Initial velocity (m/s)');

% Print the intersection points
fprintf('Initial velocity for basket: %.2f m/s\n',v1(idx));
fprintf('Initial velocity for building: %.2f m/s\n',v2(idx));
fprintf('Theta: %.2f degrees\n',theta(idx));
