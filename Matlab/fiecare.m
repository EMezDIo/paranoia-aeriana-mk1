% Real-Time Controller Graph
clear; clc;
sensivity=0.5;
savedRPS = 0;

initial_altitude = 300;
altitude = initial_altitude;

Ct=0.15;
propDiam=(5*0.0256);
G=11.768;

threshold = 0.2;

FL = 0;
FR = 0;
RL = 0;
RR = 0;

tilt = 0;
pitch = 0;
lift = 0;
yaw = 0;

maxTilt = 0.9;
maxPitch = 0.9;

maxFloatRPS = 0;
MaxRealRPS=27824/60; %RPM TO RPS
maxAltitude = 9000;

% 1. Setup Input (Change '1' to your device ID)
try
    joy = sim3d.io.Joystick();
catch
    error('No controller detected. Check connection or ID.');
end

% 2. Setup Figure
fig = figure('Name', 'Real-Time Controller Data', 'Color', 'black');
ax = axes('Parent', fig);

subplot(2,2,1);
h = animatedline('Color', [0.2039, 0.7882, 0.3608], 'LineWidth', 2);
ylim([-0.05 1.1]);
grid on;
subplot(2,2,2);
h2 = animatedline('Color', [0, 0.7882, 1], 'LineWidth', 2);
ylim([-0.05 1.1]);
grid on;

subplot(2,2,3);
h3 = animatedline('Color', [0, 0.7882, 1], 'LineWidth', 2);
ylim([-0.05 1.1]);
grid on;

subplot(2,2,4);
h4 = animatedline('Color', [0, 0.7882, 1], 'LineWidth', 2);
ylim([-0.05 1.1]);

grid on;
xlabel('Time');
ylabel('Controller Axis Value');


fig = figure('Name', 'Real-Time Controller Data', 'Color', 'black');
yes = axes('Parent', fig);

% --- 2x4 GRID LAYOUT (2 Rows, 4 Columns) ---

% 1. TOP-LEFT (Row 1, Columns 1 & 2)
subplot(2, 4, [1, 2]); 
trigg = animatedline('Color', [0, 0.7882, 1], 'LineWidth', 2);
xs2 = animatedline('Color', [1.0, 0.647, 0.0], 'LineWidth', 2);
xs = animatedline('Color', [0.2039, 0.7882, 0.3608], 'LineWidth', 2);
ys = animatedline('Color','r', 'LineWidth', 2);
ylim([-2.2 2.2]);
grid on;

% 2. BOTTOM-LEFT (Row 2, Columns 1 & 2)
% This maps exactly to positions 5 and 6 as requested
subplot(2, 4, [5, 6]); 
motorCoeff = animatedline('Color', [1, 0, 1], 'LineWidth', 2);
ylim([-2.2 2.2]);
grid on;

% 3. TOP-RIGHT (Row 1, Columns 3 & 4)
subplot(2, 4, [3, 7]); 
verify = animatedline('Color', [0, 0.7882, 1], 'LineWidth', 2);
ylim([-0.1, 10000]);
grid on;

% 4. BOTTOM-RIGHT (Row 2, Columns 3 & 4)
subplot(2, 4, [4, 8]); 
newLine = animatedline('Color', [1, 1, 0], 'LineWidth', 2); 
ylim([-1.1, MaxRealRPS+10]);
grid on;



% 3. Real-Time Loop
startTime = datetime('now');
stopPolling = false;





% Loop until figure is closed
while ishandle(fig)
    joyValues = axis(joy);
    Xaxis = joyValues(1);
    Yaxis = -1*joyValues(2);
    ChangeAltitude = -1*joyValues(3);
    Xaxis2 = joyValues(4);
    t = seconds(datetime('now') - startTime);
    
    %if sequence for a limited use of TILT between a minimum and a maximum
    if(abs(Xaxis)>threshold)
        if(Xaxis>0)
            if(Xaxis-threshold>maxTilt)
                tilt = maxTilt;
            else
                tilt = Xaxis-threshold;
            end
        else
            if(Xaxis+threshold<-1*maxTilt)
                tilt = -1*maxTilt;
            else
                tilt = Xaxis+threshold;
            end
        end
    else
        tilt = 0;
    end
    %if sequence for a limited use of PITCH between a minimum and a maximum
    if(abs(Yaxis)>threshold)
        if(Yaxis>0)
            if(Yaxis-threshold>maxPitch)
                pitch = maxPitch;
            else
                pitch = Yaxis-threshold;
            end
        else
            if(Yaxis+threshold<-1*maxPitch)
                pitch = -1*maxPitch;
            else
                pitch= Yaxis+threshold;
            end
        end
    else
        pitch = 0;
    end

    if(abs(Xaxis2)>threshold)
        yaw = Xaxis2;
    else
        yaw = 0;
    end
    
    altitude = altitude + sensivity^7*(ChangeAltitude*1000);
    if(altitude < 0)
        altitude = 0;
    end
    [~,~,~,airDensity] = atmosisa(altitude);

    FloatRPS = sqrt((G/4)/(Ct*airDensity*propDiam^4));
    if(abs(ChangeAltitude)>FloatRPS/MaxRealRPS && FloatRPS/MaxRealRPS<=1)
        lift = ChangeAltitude;
    else
        lift = FloatRPS/MaxRealRPS;
    end
    

    
    %adding all the moves coefincients to the motors
    FL = lift - pitch + tilt + yaw;
    RL = lift + pitch + tilt - yaw;
    FR = lift - pitch - tilt - yaw;
    RR = lift + pitch - tilt + yaw;
    
    motors = [FL,RL,FR,RR];
    
    %making sure there are no values <0 and if there are all of them are
    %greater than 0 so that no motor would stop rotating
    noStop = 0;
    for i=1:1:4
        if(motors(i)<0)
            motors(i) = 0;
            noStop=1;
        end
    end
    if(noStop==1)
        motors(i)=motors(i)+0.05;
    end
    
    %interpolating the values
    maximum = max(motors);
    if(maximum>1)
        motors = motors/maximum;
    end

    %all motors then can be multiplied by the maxRealRPS var to get the rotation 
    
    %END OF LOGICAL CODE

    %Start of TELEMETRY

    addpoints(h,t,motors(1));  %FL
    addpoints(h2,t,motors(3)); %FR
    addpoints(h3,t,motors(2)); %RL
    addpoints(h4,t,motors(4)); %RR


    addpoints(xs,t,Xaxis); % left joystick x-axis
    addpoints(ys,t,Yaxis); % left joystick y-axis
    addpoints(trigg,t,2*ChangeAltitude); % triggers LT(down) and RT(up)
    addpoints(verify,t,altitude);
    addpoints(newLine,t,FloatRPS);
    addpoints(motorCoeff,t,lift);
    addpoints(xs2,t,1.5*Xaxis2); % right joystick x-axis

    
    xlim(h.Parent, [t-10, t]);
    xlim(h2.Parent, [t-10, t]);
    xlim(h3.Parent, [t-10, t]);
    xlim(h4.Parent, [t-10, t]);


    xlim(trigg.Parent, [t-10, t]);
    xlim(verify.Parent, [t-10,t]);
    xlim(newLine.Parent, [t-10,t]);
    xlim(motorCoeff.Parent, [t-10,t]);

    drawnow limitrate;
end