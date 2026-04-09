% Real-Time Controller Graph
clear; clc;

labels = {'A', 'B', 'X', 'Y', 'L-Trigger', 'R-Trigger'};
colors = [
    92,  201, 52;   % Green (A)
    255, 50,  50;   % Red (B)
    50,  150, 255;  % Blue (X)
    255, 215, 0;    % Yellow (Y)
    150, 150, 150;  % Gray (LT)
    255, 120, 0     % Orange (RT)
] / 255;



% 1. Setup Input (Change '1' to your device ID)
try
    joy = sim3d.io.Joystick();
catch
    error('No controller detected. Check connection or ID.');
end

% 2. Setup Figure
fig = figure('Name', 'Real-Time Controller Data', 'Color', 'w');
ax = axes('Parent', fig);
h = gobjects(1, length(labels)); 

figure('Color', 'w'); hold on; grid on;
for i = 1:length(labels)
    h(i) = animatedline('Color', colors(i,:), 'LineWidth', 1.5, 'DisplayName', labels{i});
end
legend('Location', 'eastoutside');

grid on;
xlabel('Time');
ylabel('Controller Axis Value');
ylim([-1.1 1.1]); % Standard joystick range

% 3. Real-Time Loop
startTime = datetime('now');
stopPolling = false;

% Loop until figure is closed
while ishandle(fig)
    
    joyValues = axis(joy);
    i = 1;
    
    % Calculate elapsed time
    t = seconds(datetime('now') - startTime);
    
    % Update Graph
    addpoints(h, t, Xaxis);


    % Scroll the X-axis to follow the data
    if t > 10
        xlim(ax, [t-10, t]);
    end
    
    % Force MATLAB to render the graphic
    drawnow limitrate;
end