clear; clc;



% Real-Time Controller Graph


% 1. Setup Input (Change '1' to your device ID)
try
    joy = sim3d.io.Joystick();
catch
    error('No controller detected. Check connection or ID.');
end

% 2. Setup Figure
fig = figure('Name', 'Real-Time Controller Data', 'Color', 'black');
ax = axes('Parent', fig);
subplot(2,3,1);
grid on;
FL = animatedline('Color', [0.2039, 0.7882, 0.3608], 'LineWidth', 2);
subplot(2,3,2);
grid on;
FR = animatedline('Color', [0, 0.7882, 1], 'LineWidth', 2);
subplot(2,3,4);
grid on;
RL = animatedline('Color', [1, 0.549019, 0], 'LineWidth', 2);
subplot(2,3,5);
grid on;
RR = animatedline('Color', [1, 0,0], 'LineWidth', 2);

grid on;
subplot(2,3,3);
D_altitude = animatedline('Color', [1, 0,0], 'LineWidth', 2);


xlabel('Time');
ylabel('Newtons');


m = 10^-3*1200; %grams to kg
Ct=0.15;
Cd=1.3;

A = 0.02;

base_altitude=300;

MaxRealRPS=27824/60;

propDiam=(5*0.0256);

threshold = 0.2;

G=11.768;

FL_thrust=0;
FR_thrust=0;
RL_thrust=0;
RR_thrust=0;

FL_RPS=0;
FR_RPS=0;
RL_RPS=0;
RR_RPS=0;


startTime = datetime('now');
stopPolling = false;

sensivity=0.5;
savedRPS = 0;

intialPosition = 300;
currentPosition = intialPosition;
D_velo = 0;
timeAltitude = 0.01;
dragForce = 0;


ylim(FL.Parent, [-0.5, 12]);
ylim(RL.Parent, [-0.5, 12]);
ylim(FR.Parent, [-0.5, 12]);
ylim(RR.Parent, [-0.5, 12]);


acc=0;
% Loop until figure is closed
while ishandle(fig)
    joyValues = axis(joy);
    trigger = -1*joyValues(3);
    LJXaxis = joyValues(1);
    LJYaxis = joyValues(2);
    
    savedRPS=savedRPS+trigger*sensivity;
    if(savedRPS<0)
        savedRPS=0;
    end
    if(savedRPS>MaxRealRPS)
            savedRPS=MaxRealRPS;
    end

    if(abs(LJXaxis)>threshold)
        FL_RPS = FL_RPS + LJYaxis * sensivity;
        RL_RPS = RL_RPS + LJYaxis * sensivity;
        FR_RPS = FR_RPS - LJYaxis * sensivity;
        RR_RPS = RR_RPS - LJYaxis * sensivity;
    end
    if(abs(LJYaxis)>threshold)
        FL_RPS = FL_RPS - LJYaxis * sensivity;
        FR_RPS = FR_RPS - LJYaxis * sensivity;
        RL_RPS = RL_RPS + LJYaxis * sensivity;
        RR_RPS = RR_RPS + LJYaxis * sensivity;
    end

    [~, ~, ~, airDensity] = atmosisa(currentPosition);

    Vertical_thrust = Ct * airDensity * savedRPS^2 * propDiam^4;
    MaximumThrust = Ct * airDensity * MaxRealRPS^2 * propDiam^4;
    
    Thrust_FL = min(FL_thrust+Vertical_thrust, MaximumThrust);
    Thrust_FR = min(FR_thrust+Vertical_thrust, MaximumThrust);
    Thrust_RL = min(RL_thrust+Vertical_thrust, MaximumThrust);
    Thrust_RR = min(RR_thrust+Vertical_thrust, MaximumThrust);
    Total_Thrust = Thrust_FL + Thrust_FR + Thrust_RL + Thrust_RR;
    
   
    dragFactor = 0.5 * airDensity * Cd * A;
    dragForce = dragFactor * D_velo * abs(D_velo); 
    acc = (Total_Thrust - G - dragForce) / m;
    D_velo = D_velo + (acc * timeAltitude);
    currentPosition = currentPosition + (D_velo * timeAltitude);
    
    if (currentPosition < 0)
        currentPosition = 0;
        D_velo = 0; 
    end

    t = seconds(datetime('now') - startTime);

    addpoints(FL, t, Thrust_FL);
    addpoints(FR, t, Thrust_FR);
    addpoints(RL, t, Thrust_RL);
    addpoints(RR, t, Thrust_RR);

    addpoints(D_altitude,t,currentPosition);

    xlim(FL.Parent, [t-10, t]);
    xlim(RL.Parent, [t-10, t]);
    xlim(FR.Parent, [t-10, t]);
    xlim(RR.Parent, [t-10, t]);
    xlim(D_altitude.Parent, [t-10, t]);
    % Force MATLAB to render the graphic
    drawnow limitrate;
end