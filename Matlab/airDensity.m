% Define the altitude range from -500m to 9000m
% (10-meter increments)
altitude = 0:10:2500; 

% Use atmosisa to calculate properties
% Returns Temperature (T), Speed of Sound (a), Pressure (P), and Density (rho)
[T, a, P, rho] = atmosisa(altitude);

% Plot Altitude vs. Air Density
figure;
plot(rho, altitude, 'LineWidth', 2);
grid on;
title('Air Density vs. Altitude (ISA Model)');
xlabel('Air Density (kg/m^3)');
ylabel('Altitude (meters)');

% Add reference lines for Sea Level and Mt. Everest
yline(0, '--', 'Sea Level', 'LabelHorizontalAlignment', 'left');
yline(8848, '--', 'Mt. Everest', 'LabelHorizontalAlignment', 'left');



