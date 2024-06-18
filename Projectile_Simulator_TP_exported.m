classdef Projectile_Simulator_TP_exported < matlab.apps.AppBase

    % Properties that correspond to app components
    properties (Access = public)
        UIFigure                    matlab.ui.Figure
        Versionlabel                matlab.ui.control.Label
        ProjectileSimulatorLabel    matlab.ui.control.Label
        TabGroup                    matlab.ui.container.TabGroup
        simulator                   matlab.ui.container.Tab
        simlayout                   matlab.ui.container.GridLayout
        InputPanel                  matlab.ui.container.Panel
        Distance                    matlab.ui.control.NumericEditField
        Height                      matlab.ui.control.NumericEditField
        acceleration                matlab.ui.control.DropDown
        RTSSSwitch                  matlab.ui.control.Switch
        simulate                    matlab.ui.control.Button
        SaveButton                  matlab.ui.control.Button
        resetall                    matlab.ui.control.Button
        RTSSSwitchLabel             matlab.ui.control.Label
        mLabel_3                    matlab.ui.control.Label
        mLabel                      matlab.ui.control.Label
        RangeLabel                  matlab.ui.control.Label
        IntialHeightLabel           matlab.ui.control.Label
        AccelerationLabel           matlab.ui.control.Label
        OutputPanel                 matlab.ui.container.Panel
        TimePeak                    matlab.ui.control.EditField
        secLabel_3                  matlab.ui.control.Label
        RangeLabel_3                matlab.ui.control.Label
        TimeatPeakLabel             matlab.ui.control.Label
        mLabel_5                    matlab.ui.control.Label
        velocity                    matlab.ui.control.NumericEditField
        VelocityLabel               matlab.ui.control.Label
        timeofflight                matlab.ui.control.NumericEditField
        TimeofflightLabel           matlab.ui.control.Label
        MH                          matlab.ui.control.NumericEditField
        MaxHieghtLabel              matlab.ui.control.Label
        peak                        matlab.ui.control.EditField
        PeakLocationEditFieldLabel  matlab.ui.control.Label
        angle                       matlab.ui.control.NumericEditField
        AngleofLaunchLabel          matlab.ui.control.Label
        Range                       matlab.ui.control.EditField
        RangeLabel_2                matlab.ui.control.Label
        msLabel                     matlab.ui.control.Label
        secLabel_2                  matlab.ui.control.Label
        mLabel_2                    matlab.ui.control.Label
        radsLabel                   matlab.ui.control.Label
        mLabel_4                    matlab.ui.control.Label
        LocationTrackerPanel        matlab.ui.container.Panel
        timelocate                  matlab.ui.control.NumericEditField
        locate                      matlab.ui.control.Button
        secLabel                    matlab.ui.control.Label
        locationdisp                matlab.ui.control.TextArea
        CheckobjectlocationatLabel  matlab.ui.control.Label
        RTSSPanel                   matlab.ui.container.Panel
        TimeElapsedLabel            matlab.ui.control.Label
        TimeElapsedValue            matlab.ui.control.Label
        TimeRemainingLabel          matlab.ui.control.Label
        TimeRemainingValue          matlab.ui.control.Label
        CurrentHeightLabel          matlab.ui.control.Label
        CurrentHeightValue          matlab.ui.control.Label
        DistanceCoveredLabel        matlab.ui.control.Label
        DistanceCoveredValue        matlab.ui.control.Label
        DistanceRemainingLabel      matlab.ui.control.Label
        DistanceRemainingValue      matlab.ui.control.Label
        ReplayButton                matlab.ui.control.Button
        FastRewindButton            matlab.ui.control.Button
        Skipback                    matlab.ui.control.Button
        PlayPauseButton             matlab.ui.control.StateButton
        Skipforward                 matlab.ui.control.Button
        FastForwardButton           matlab.ui.control.Button
        InputPlaybackSpeed          matlab.ui.control.Spinner
        MainGraph                   matlab.ui.control.UIAxes
        GraphsTab                   matlab.ui.container.Tab
        GridLayout                  matlab.ui.container.GridLayout
        Jerk                        matlab.ui.control.NumericEditField
        MaxDisplacementLabel_10     matlab.ui.control.Label
        MinAcceleration             matlab.ui.control.NumericEditField
        MaxDisplacementLabel_9      matlab.ui.control.Label
        MaxAcceleration             matlab.ui.control.NumericEditField
        MaxDisplacementLabel_8      matlab.ui.control.Label
        AvgAcceleration             matlab.ui.control.NumericEditField
        MaxDisplacementLabel_7      matlab.ui.control.Label
        MinVelocity                 matlab.ui.control.NumericEditField
        MaxDisplacementLabel_5      matlab.ui.control.Label
        MaxVelocity                 matlab.ui.control.NumericEditField
        MaxDisplacementLabel_4      matlab.ui.control.Label
        AvgVelocity                 matlab.ui.control.NumericEditField
        MaxDisplacementLabel_3      matlab.ui.control.Label
        MinDisplacement             matlab.ui.control.NumericEditField
        MaxDisplacementLabel_2      matlab.ui.control.Label
        MaxDisplacement             matlab.ui.control.NumericEditField
        MaxDisplacementLabel        matlab.ui.control.Label
        DisplacementTimeGraph       matlab.ui.control.UIAxes
        VelocityTimeGraph           matlab.ui.control.UIAxes
        AccelerationTimeGraph       matlab.ui.control.UIAxes
        HistoryTab                  matlab.ui.container.Tab
        GridLayout2                 matlab.ui.container.GridLayout
        TabGroup2                   matlab.ui.container.TabGroup
        MainTab                     matlab.ui.container.Tab
        ClearAllButton              matlab.ui.control.Button
        DataDropDown                matlab.ui.control.DropDown
        DataDropDownLabel           matlab.ui.control.Label
        ImportButton                matlab.ui.control.Button
        SimulationH                 matlab.ui.control.Table
        Historygraph                matlab.ui.control.UIAxes
    end

    properties (Access = private)
        X % Main - X Values
        Y % Main - Y Values
        T % Main Time
        TMAX % Maximum Time

        sAni = true % Should the program animate
        isAni = true % Is the program animating
        isPlay = true % Is the program playing the animation    
        isUpdat = true % Is the program updating
        cTime = 0 % Start Time
        pSpeed = 1 % Playback speed
        timerr = timer('ExecutionMode','fixedRate', ...
            'Period', 0.1, ...
            'TimerFcn', @(~,~) updateRTSS(app));
        hisname = [] % Array of data for importing in graph (History Tab)
        playIcon = 'Play.png' % Play Icon
        pauseIcon = 'Pause.png' % Pause Icon
        new_tab = [] % TODO
    end

    methods (Access = public)
        % Simulation Tab

        % This is the input function of the program
        function [x, H, y_b, d, D, g] = inputs(app)
            x = app.Distance.Value; % Distance from obstacle
            H = app.Height.Value; % Height of obstacle
            a = app.acceleration.Value; % Acceleration ie. Gravity
            y_b = 3;    % Height of the basket in meters
            d = 6;      % Distance from obstacle to basket in meters
            D = x + d;  % Total horizontal distance from thrower to basket
            if a == "Earth"
                g = 9.81; % Earth Gravity
            elseif a == "Moon"
                g = 1.62; % Moon Gravity
            elseif a == "Mars"
                g = 3.71; % Mars Gravity
            end
        end

        % Define the system of equations to solve
        function F = eqnss(app, vars)
            [x, H, y_b, ~, D, g] = inputs(app); % Get required inputs
            alpha = vars(1); % Angle initial guess
            v0 = vars(2); % Velocity initial guess
            safety_margin = 1e-6; % Margin to avoiid hitting obstacle

            % Equation for the vertical position at the basket
            eq1 = y_b - (D * tan(alpha) - (1/2) * g * (D / (v0 * cos(alpha)))^2);

            % Equation for the height at the obstacle
            eq2 = H + safety_margin - (x * tan(alpha) - (1/2) * g * (x / (v0 * cos(alpha)))^2);

            % Return the system of equations
            F = [eq1; eq2];
        end

        function [V, alpha, tofl, rangeh, t_max_height, max_height, x_max_height, peakvalue] = calculate(app, D, g)

            % Initial guesses for alpha and v0 1e-6 degrees and 1e-6 m/s
            initial_guess = [(1e-6) * pi / 180, 1e-6];

            % Generate function for fsolve
            fun = @(initial_guess)eqnss(app, initial_guess);

            % Solve the system of equations using fsolve
            % MaxFunEvals - How many times can the function be checked with values
            % MaxIterations - How many iterations of paramter can be made
            options = optimoptions('fsolve', 'MaxFunEvals', 1e+50, 'MaxIterations', 1e+50);
            solution = fsolve(fun, initial_guess, options); % fsolve function with initial guess and extra options

            % Extract the solutions
            alpha = abs(solution(1)); % Angle
            V = abs(solution(2)); % Velocity

            tofl = D / (V * cos(alpha)); % Time of flight
            rangeh = D; % Max X
            t_max_height = V * sin(alpha) / g; % Time at max height
            max_height = V * sin(alpha) * t_max_height - 0.5 * g * t_max_height^2; % Max height
            x_max_height = V * cos(alpha) * t_max_height ; % Max Height X - Coordinates % X coordinate at max height
            peakvalue = ['(',num2str(round(x_max_height, 2)),',',num2str(round(max_height, 2)),')'];
        end

        function plotgraph(app) % Plots the static Graph
            [~, ~, ~, ~, D, g] = inputs(app);
            [V, alpha, tofl, ~, ~, ~, ~, ~] = calculate(app, D, g);
            t = 0:1e-5:tofl; % Time array
            x = V * cos(alpha) * t; % X axis - Horizontal Displacement
            y = V * sin(alpha) * t - 0.5 * g * t.^2; % Y Axis - Vertical Displacement
            plot(app.MainGraph,x,y)
        end

        function updateRTSS(app) % This functions Updates the Real time animation of the RTSS
            if app.sAni
                if app.isAni && all(app.cTime >= 0) && all(app.cTime <= (app.TMAX + 1))
                    index = find(app.T <= app.cTime, 1, 'last');

                    if ~isempty(index)
                        x = app.X(index);
                        y = app.Y(index);
                        updateRTSSv(app, x, y, app.T(index));

                        plot(app.MainGraph, x, y, 'o', 'MarkerFaceColor', 'red');
                        hold(app.MainGraph, 'on');
                        plot(app.MainGraph, app.X(1:index), app.Y(1:index), '-');

                        xlim(app.MainGraph, [min(app.X), max(app.X)]);
                        ylim(app.MainGraph, [min(app.Y), max(app.Y)]);

                        drawnow;
                        hold(app.MainGraph, 'off');

                        if app.isUpdat
                            app.cTime = app.cTime + (app.pSpeed / 4);
                        else
                            app.isUpdat = true;
                        end
                    end
                else
                    stop(app.timerr);
                    app.isAni = false;
                end
            end
        end

        function pauseRTSS(app) % This function pauses the animation
            app.isPlay = false;
            app.isUpdat = false;
            stop(app.timerr)
            app.PlayPauseButton.Icon = app.playIcon;
            app.timerr.running
        end

        function resumeRTSS(app) % This function resumes the animation
            app.isPlay = true;
            start(app.timerr);
            app.PlayPauseButton.Icon = app.pauseIcon;
        end

        function startRTSS(app) % This function starts the RTSS
            app.cTime = 0;
            app.isAni = true;
            app.isPlay = true;
            app.PlayPauseButton.Icon = app.pauseIcon;
            app.timerr = timer('ExecutionMode','fixedRate', ...
                'Period', 0.1, ...
                'TimerFcn', @(~,~) updateRTSS(app));
            start(app.timerr);
        end

        function updateRTSSv(app, x, y, t) % This function updates the values of the RTSS
            distanceRemaining = max(app.X) - x;
            timeRemaining = app.TMAX - t;

            app.CurrentHeightValue.Text = sprintf('%.2f m', y);
            app.DistanceCoveredValue.Text = sprintf('%.2f m', x);
            app.DistanceRemainingValue.Text = sprintf('%.2f m', distanceRemaining);
            % app.CurrentVelocityValue.Text = sprintf('%.2f m/s', currentVelocity);
            app.TimeElapsedValue.Text = sprintf('%.2f s', t);
            app.TimeRemainingValue.Text = sprintf('%.2f s', timeRemaining);
        end

        function tweakRTSS(app, pspeed) % This function tweaks the speed of the animation
            if app.isAni
                pauseRTSS(app);
                app.cTime = app.cTime + (app.pSpeed / pspeed);
                if app.cTime < 0
                    app.cTime = 0;
                end
                updateRTSS(app);
            end
        end

        function rtssenable(app) % This function enables or disables the RTSS
            rs = app.RTSSSwitch.Value;
            if strcmp(rs,'On') % Check the RTSS switch if it is ON and enables the RTSS panel
                app.RTSSPanel.Enable = "on";
                app.ReplayButton.Enable = "on";
                app.FastForwardButton.Enable = "on";
                app.FastRewindButton.Enable= "on";
                app.ReplayButton.Enable= "on";
                app.Skipback.Enable= "on";
                app.PlayPauseButton.Enable = "on";
                app.Skipforward.Enable= "on";
                app.InputPlaybackSpeed.Enable= "on";
                startRTSS(app);
                app.sAni = true;
            else % If off then disables the panel along with its buttons
                app.sAni = false;
                app.plotgraph;
                app.RTSSPanel.Enable = "off";
                app.ReplayButton.Enable = "off";
                app.FastForwardButton.Enable = "off";
                app.FastRewindButton.Enable= "off";
                app.ReplayButton.Enable= "off";
                app.Skipback.Enable= "off";
                app.PlayPauseButton.Enable = "off";
                app.Skipforward.Enable= "off";
                app.InputPlaybackSpeed.Enable= "off";
            end
        end

        % Graph Tab

        % This function plots displacement against time in graphs tab
        function plotdisp(app)
            [~, ~, ~, ~, D, g] = inputs(app);
            [V, alpha, tofl, ~, ~, ~, ~, ~] = calculate(app, D, g);
            x = linspace(0, tofl, 1e+5); % Time array
            y = V * sin(alpha) * x - 0.5 * g * x.^2; % Displacement based on time
            plot(app.DisplacementTimeGraph, x, y)
            app.MaxDisplacement.Value = max(y); % Max Displacement
            app.AvgVelocity.Value = V * cos(alpha); % Average Velocity
        end

        % This fucntions plots velocity against time in the graphs tab
        function plotvel(app)
            [~, ~, ~, ~, D, g] = inputs(app);
            [V, ~, tofl, ~, ~, ~, ~, ~] = calculate(app, D, g);
            x = linspace(0, tofl, 1e+5); % Time array
            y = linspace(-V, 0, 1e+5); % Velocity array
            plot(app.VelocityTimeGraph, x, y)
            app.MinVelocity.Value = -V; % Max velocity
            app.AvgAcceleration.Value = -g; % Average acceleration
        end

        % This function plots acceleration against time in the graphs tab
        function plotacc(app)
            [~, ~, ~, ~, D, g] = inputs(app);
            [~, ~, tofl, ~, ~, ~, ~, ~] = calculate(app, D, g);
            x = linspace(0, tofl, 1e+5); % Time array
            y = linspace(-g, -g, 1e+5); % Acceleration array
            plot(app.AccelerationTimeGraph, x, y)
            app.MaxAcceleration.Value = -g; % Max Acceleration
            app.Jerk.Value = 0; % Jerk = 0 since dy/dt(constant)
        end

        % History Tab
        function importf(app)
            db = app.SimulationH.Data;
            name = app.DataDropDown.Value;
            index = app.DataDropDown.ValueIndex;
            val = db(index,:);
            D = cell2mat(val(2));
            g = cell2mat(val(4));
            [~, ~, tofl, rangeh, ~, max_height, ~, peakvalue] = calculate(app, D, g);
            newtab(app,name,tofl,rangeh,max_height,peakvalue)
        end

        function newtab(app,name,tofh,rangeh,maxh,peakvalue)
            tabname = uitab(app.TabGroup2,"Title",name);
            app.new_tab = [app.new_tab, tabname];
            % Create TimeofflightLabel_2
            TimeofflightLabel_tab = uilabel(tabname);
            TimeofflightLabel_tab.Position = [8 278 78 22];
            TimeofflightLabel_tab.Text = 'Time of flight:';
            Timeofflight = uieditfield(tabname, 'text');
            Timeofflight.Editable = 'off';
            Timeofflight.HorizontalAlignment = 'center';
            Timeofflight.Value = num2str(tofh);
            Timeofflight.Position = [92 278 100 22];

            % Create MaxHeightLabel_2
            MaxHeightLabel_tab = uilabel(tabname);
            MaxHeightLabel_tab.Position = [8 215 70 22];
            MaxHeightLabel_tab.Text = 'Max Height:';

            % Create MH_2
            MH_tab = uieditfield(tabname, 'numeric');
            MH_tab.Editable = 'off';
            MH_tab.HorizontalAlignment = 'center';
            MH_tab.Position = [92 215 101 22];
            MH_tab.Value = maxh ;

            % Create RangeLabel_2
            RangeLabel_tab = uilabel(tabname);
            RangeLabel_tab.Position = [10 183 43 22];
            RangeLabel_tab.Text = 'Range:';

            % Create range_2
            range_tab = uieditfield(tabname, 'numeric');
            range_tab.Editable = 'off';
            range_tab.HorizontalAlignment = 'center';
            range_tab.Position = [92 183 100 22];
            range_tab.Value = rangeh;

            % Create PeakLocationEditFieldLabel_2
            PeakLocationEditFieldLabel_tab = uilabel(tabname);
            PeakLocationEditFieldLabel_tab.Position = [8 247 85 22];
            PeakLocationEditFieldLabel_tab.Text = 'Peak Location:';

            % Create peak_2
            peak_tab = uieditfield(tabname, 'text');
            peak_tab.Editable = 'off';
            peak_tab.HorizontalAlignment = 'center';
            peak_tab.Placeholder = '(x,y)';
            peak_tab.Position = [93 247 100 22];
            peak_tab.Value = peakvalue;
        end

        function plothis(app)
            index = app.DataDropDown.ValueIndex;
            db = app.SimulationH.Data;
            val = db(index,:);
            name = cellstr(val(1));
            g= cell2mat(val(4));
            V= cell2mat(val(5));
            alpha = cell2mat(val(6));
            tofl = cell2mat(val(7));
            t = linspace(0, tofl, 100); % Time array
            x = V * cos(alpha) * t; % X - Axis
            y = V * sin(alpha) * t - 0.5 * g * t.^2; % Y - Axis
            app.hisname = [app.hisname; name];
            plot(app.Historygraph,x,y)
            legend(app.Historygraph,app.hisname)
            hold(app.Historygraph,"on")
        end
    end

    % Callbacks that handle component events
    methods (Access = private)

        % Button pushed function: simulate
        function simulateButtonPushed(app, event)
            [x, H, y_b, ~, D, g] = inputs(app);
            isValidDistance = true; % Initialize checking variable - Distance
            isValidHeight = true; % Initialize checking variable - Height

            % Error if distance from building is too less
            if x < 0.01
                uialert(app.UIFigure, ["Too close to the building!", "Try a larger value"], "Too close", "Icon", "error");
                isValidDistance = false;
            end

            % Error if obstacle height less than final height
            if H < y_b
                uialert(app.UIFigure, "Obstacle cannot be shorter than final height!", "Obstacle too short", "Icon", "error");
                isValidHeight = false;
            end

            % Check if both conditions are met
            if isValidDistance && isValidHeight
                [V, alpha, tofl, ~, t_max_height, max_height, ~, peakvalue] = calculate(app, D, g);

                % Error if angle greater than 89.99
                if round(rad2deg(alpha), 2) > 89.99
                    uialert(app.UIFigure, "Angle is too large!", "Angle too large", "Icon", "error");
                else
                    app.velocity.Value = round(V, 2) ; % Displays Velocity
                    app.angle.Value = round(alpha, 2); % Displays Angle
                    app.timeofflight.Value = round(tofl, 2); % Displays Time of flight

                    rtssenable(app); % RTSS Enable

                    app.MH.Value = max_height; % Maximum Height
                    app.peak.Value = peakvalue; % Peak value (x,y)
                    app.Range.Value = num2str(D); % Total range
                    app.TimePeak.Value = num2str(round(t_max_height, 2)); % Time at Peak Value

                    plotdisp(app); % Plot displacement time graph
                    plotvel(app); % Plot velocity time graph
                    plotacc(app); % Plot displacement time graph

                    app.locate.Enable ="on"; % Enables the locate button
                    app.timelocate.Enable = "on"; % Enables the Time field
                    app.timelocate.Editable = "on"; % Makes time field editable
                    app.secLabel.Enable ="on";

                    t = 0:1e-5:tofl; % Time array
                    x = V * cos(alpha) * t; % X - Axis
                    y = V * sin(alpha) * t - 0.5 * g * t.^2; % Y - Axis

                    y(y < 0) = 0; % Condition to make sure Y is 0
                    app.X = x; % Sets RTSS X Array
                    app.Y = y; % Sets RTSS Y Array
                    app.T = t; % Sets RTSS T Array
                    app.TMAX = tofl; % Sets RTSS TMAX

                    app.SaveButton.Enable = "on"; % Enables Save button
                end
            end
        end

        % Button pushed function: resetall
        function resetallButtonPushed(app, event)
            % Reset all values and set to default

            % Input Panel
            app.Distance.Value = 0;
            app.Height.Value = 0;
            app.acceleration.Value = "Earth";
            app.RTSSSwitch.Value = "Off";

            % Output Panel
            app.velocity.Value = 0;
            app.timeofflight.Value = 0;
            app.MH.Value = 0;
            app.angle.Value = 0;
            app.Range.Value = "0";
            app.TimePeak.Value = "0";
            app.peak.Value = "(x,y)";

            % Main Graph
            plot(app.MainGraph,0,0)

            % Location Tracker
            app.timelocate.Value = 0;
            app.locationdisp.Value = "At time t s, the object is at position (x, y)";
            app.timelocate.Editable = "off";
            app.locate.Enable ="off";

            % RTSS Panel
            app.RTSSPanel.Enable = "off";
            app.ReplayButton.Enable = "off";
            app.FastForwardButton.Enable = "off";
            app.FastRewindButton.Enable= "off";
            app.ReplayButton.Enable= "off";
            app.Skipback.Enable= "off";
            app.PlayPauseButton.Enable = "off";
            app.Skipforward.Enable= "off";
            app.InputPlaybackSpeed.Enable= "off";

            % Graphs Tab
            % Graphs
            plot(app.DisplacementTimeGraph,0,0)
            plot(app.VelocityTimeGraph,0,0)
            plot(app.AccelerationTimeGraph,0,0)

            % Values under graph
            app.MaxAcceleration.Value = 0;
            app.MinAcceleration.Value = 0;
            app.AvgAcceleration.Value = 0;
            app.Jerk.Value = 0;
            app.MaxVelocity.Value = 0;
            app.MinVelocity.Value = 0;
            app.AvgVelocity.Value = 0;
            app.MinDisplacement.Value = 0;
            app.MaxDisplacement.Value = 0;
        end

        % Button pushed function: locate
        function locateButtonPushed(app, event)
            [~, ~, ~, ~, D, g] = inputs(app);
            [V, alpha, tofl, ~, ~, ~, ~, ~] = calculate(app, D, g);
            t_check = app.timelocate.Value; % Time field
            if t_check >= tofl % If input time greater than total time
                x_check = V * cos(alpha) * tofl;
                app.locationdisp.Value= ['At time ' num2str(round(t_check, 2)) ' s, the object is at position (' num2str(round(x_check, 2)) ', 3)'];  % Displays time, (x,y) coordinates in locationdisp if time input is greater or equal to time of flight
            else
                x_check = V * cos(alpha) * t_check; % X - Location
                y_check = V * sin(alpha) * t_check - 0.5 * g * t_check^2; % Y - Location
                app.locationdisp.Value= ['At time ' num2str(round(t_check, 2)) ' s, the object is at position (' num2str(round(x_check, 2)) ', ' num2str(round(y_check, 2)) ')'];  % Displays time, (x,y) coordinates in locationdisp
            end
        end

        % Button pushed function: ReplayButton
        function ReplayButtonPushed(app, event)
            if app.isAni
                pauseRTSS(app);
                app.cTime = 0;
                resumeRTSS(app);
            else
                startRTSS(app);
            end
        end

        % Button pushed function: FastRewindButton
        function FastRewindButtonPushed(app, event)
            tweakRTSS(app, -4);
        end

        % Button pushed function: Skipback
        function SkipbackPushed(app, event)
            if app.isAni
                pauseRTSS(app);
                app.cTime = 0;
                resumeRTSS(app);
            else
                startRTSS(app);
            end
        end

        % Value changed function: PlayPauseButton
        function PlayPauseButtonValueChanged(app, event)
            if app.isAni
                if app.isPlay
                    pauseRTSS(app);
                    app.timerr.Running
                else
                    resumeRTSS(app);
                    app.timerr.Running
                end
            end
        end

        % Button pushed function: Skipforward
        function SkipforwardPushed(app, event)
            if app.isAni
                pauseRTSS(app);
                app.cTime = app.TMAX;
                resumeRTSS(app);
            else
                startRTSS(app);
            end
        end

        % Button pushed function: FastForwardButton
        function FastForwardButtonPushed(app, event)
            tweakRTSS(app, 2);
        end

        % Value changed function: InputPlaybackSpeed
        function InputPlaybackSpeedValueChanged(app, event)
            app.pSpeed = app.InputPlaybackSpeed.Value;
        end

        % Button pushed function: SaveButton
        function SaveButtonPushed(app, event)
            s = inputdlg("Enter the name of the Data","Save");
            if isempty(s)
                uialert(app.UIFigure, "Save Cancelled", "Failure", "Icon", "warning");
            else
                [~, H, ~, ~, D, g] = inputs(app);
                [V, alpha, tofl, ~, ~, ~, ~, ~] = calculate(app, D, g);
                data = [s, D, H, round(g, 2), round(V, 2), round(alpha, 2), round(tofl, 2)];
                sh = app.SimulationH.Data;
                app.SimulationH.Data = [sh;data];
            end
        end

        % Button pushed function: ImportButton
        function ImportButtonPushed(app, event)
            importf(app);
            plothis(app);
        end

        % Button down function: HistoryTab
        function HistoryTabButtonDown(app, event)
            db = app.SimulationH.Data;
            h = height(db);
            app.DataDropDown.Items = {};
            app.DataDropDown.ItemsData ={};
            for i = 1:h
                name = db(i,1);
                ddrop = app.DataDropDown.Items;
                app.DataDropDown.Items = [ddrop name];
            end
        end

        % Button down function: simulator
        function simulatorButtonDown(app, event)
            % Need to clear the drop down menu
            app.DataDropDown.Items = {};
            app.DataDropDown.ItemsData ={};
        end

        % Button down function: GraphsTab
        function GraphsTabButtonDown(app, event)
            app.DataDropDown.Items = {};
            app.DataDropDown.ItemsData = {};
        end

        % Button pushed function: ClearAllButton
        function ClearAllButtonPushed(app, event)
            hold(app.Historygraph,"off")
            app.hisname = [];
            plot(app.Historygraph,0,0)
            legend(app.Historygraph, 'off')
            tabs = app.new_tab;
            for i  = tabs
                delete(i)
            end
        end
    end

    % Component initialization
    methods (Access = private)

        % Create UIFigure and components
        function createComponents(app)

            % Get the file path for locating images
            pathToMLAPP = fileparts(mfilename('fullpath'));

            % Create UIFigure and hide until all components are created
            app.UIFigure = uifigure('Visible', 'off');
            app.UIFigure.Color = [0.9412 0.9412 0.9412];
            app.UIFigure.Position = [100 100 957 609];
            app.UIFigure.Name = 'MATLAB App';

            % Create TabGroup
            app.TabGroup = uitabgroup(app.UIFigure);
            app.TabGroup.Position = [1 1 957 550];

            % Create simulator
            app.simulator = uitab(app.TabGroup);
            app.simulator.AutoResizeChildren = 'off';
            app.simulator.Title = 'Simulation';
            app.simulator.BackgroundColor = [0.9412 0.9412 0.9412];
            app.simulator.ButtonDownFcn = createCallbackFcn(app, @simulatorButtonDown, true);

            % Create simlayout
            app.simlayout = uigridlayout(app.simulator);
            app.simlayout.ColumnWidth = {'1.5x', '1.5x', '1x', '1x', '1x'};
            app.simlayout.RowHeight = {'1x', '1x', '1x', '1x', '1x', '1x', '1x', '1x'};
            app.simlayout.BackgroundColor = [0.9412 0.9412 0.9412];

            % Create MainGraph
            app.MainGraph = uiaxes(app.simlayout);
            title(app.MainGraph, 'Projectile Motion')
            xlabel(app.MainGraph, 'Horizontal Distance (m)')
            ylabel(app.MainGraph, 'Vertical Distance (m)')
            app.MainGraph.Toolbar.Visible = 'off';
            app.MainGraph.AmbientLightColor = [0 0 0];
            app.MainGraph.XLimitMethod = 'tight';
            app.MainGraph.YLimitMethod = 'tight';
            app.MainGraph.ZLimitMethod = 'tight';
            app.MainGraph.XColor = [0 0 0];
            app.MainGraph.YColor = [0 0 0];
            app.MainGraph.BoxStyle = 'full';
            app.MainGraph.Color = 'none';
            app.MainGraph.XGrid = 'on';
            app.MainGraph.XMinorGrid = 'on';
            app.MainGraph.YGrid = 'on';
            app.MainGraph.YMinorGrid = 'on';
            app.MainGraph.ColorOrder = [0.850980392156863 0.325490196078431 0.0980392156862745;0.850980392156863 0.325490196078431 0.0980392156862745;0.752941176470588 0.36078431372549 0.984313725490196;0.286274509803922 0.858823529411765 0.250980392156863;0.423529411764706 0.956862745098039 1;0.949019607843137 0.4 0.768627450980392];
            app.MainGraph.FontSize = 10;
            app.MainGraph.Layout.Row = [1 5];
            app.MainGraph.Layout.Column = [3 5];
            app.MainGraph.Interruptible = 'off';

            % Create RTSSPanel
            app.RTSSPanel = uipanel(app.simlayout);
            app.RTSSPanel.AutoResizeChildren = 'off';
            app.RTSSPanel.Enable = 'off';
            app.RTSSPanel.Title = 'Real Time Simulator System ( RTSS )';
            app.RTSSPanel.BackgroundColor = [0.9412 0.9412 0.9412];
            app.RTSSPanel.Layout.Row = [6 8];
            app.RTSSPanel.Layout.Column = [3 5];
            app.RTSSPanel.FontWeight = 'bold';

            % Create InputPlaybackSpeed
            app.InputPlaybackSpeed = uispinner(app.RTSSPanel);
            app.InputPlaybackSpeed.Step = 0.25;
            app.InputPlaybackSpeed.Limits = [0.25 2];
            app.InputPlaybackSpeed.ValueChangedFcn = createCallbackFcn(app, @InputPlaybackSpeedValueChanged, true);
            app.InputPlaybackSpeed.HorizontalAlignment = 'center';
            app.InputPlaybackSpeed.BackgroundColor = [0.9412 0.9412 0.9412];
            app.InputPlaybackSpeed.Enable = 'off';
            app.InputPlaybackSpeed.Tooltip = {'Set playback speed'};
            app.InputPlaybackSpeed.Position = [399 130 58 22];
            app.InputPlaybackSpeed.Value = 1;

            % Create FastForwardButton
            app.FastForwardButton = uibutton(app.RTSSPanel, 'push');
            app.FastForwardButton.ButtonPushedFcn = createCallbackFcn(app, @FastForwardButtonPushed, true);
            app.FastForwardButton.Icon = fullfile(pathToMLAPP, 'fast-forward.png');
            app.FastForwardButton.BackgroundColor = [0.9412 0.9412 0.9412];
            app.FastForwardButton.Enable = 'off';
            app.FastForwardButton.Position = [332 130 47 23];
            app.FastForwardButton.Text = '';

            % Create Skipforward
            app.Skipforward = uibutton(app.RTSSPanel, 'push');
            app.Skipforward.ButtonPushedFcn = createCallbackFcn(app, @SkipforwardPushed, true);
            app.Skipforward.Icon = fullfile(pathToMLAPP, 'Forward.png');
            app.Skipforward.IconAlignment = 'center';
            app.Skipforward.BackgroundColor = [0.9412 0.9412 0.9412];
            app.Skipforward.Enable = 'off';
            app.Skipforward.Position = [270 130 47 23];
            app.Skipforward.Text = '';

            % Create PlayPauseButton
            app.PlayPauseButton = uibutton(app.RTSSPanel, 'state');
            app.PlayPauseButton.ValueChangedFcn = createCallbackFcn(app, @PlayPauseButtonValueChanged, true);
            app.PlayPauseButton.Enable = 'off';
            app.PlayPauseButton.Icon = fullfile(pathToMLAPP, 'Play.png');
            app.PlayPauseButton.IconAlignment = 'center';
            app.PlayPauseButton.Text = '';
            app.PlayPauseButton.BackgroundColor = [0.9412 0.9412 0.9412];
            app.PlayPauseButton.Position = [233 130 25 23];

            % Create Skipback
            app.Skipback = uibutton(app.RTSSPanel, 'push');
            app.Skipback.ButtonPushedFcn = createCallbackFcn(app, @SkipbackPushed, true);
            app.Skipback.Icon = fullfile(pathToMLAPP, 'Rewind.png');
            app.Skipback.BackgroundColor = [0.9412 0.9412 0.9412];
            app.Skipback.Enable = 'off';
            app.Skipback.Position = [172 130 47 23];
            app.Skipback.Text = '';

            % Create FastRewindButton
            app.FastRewindButton = uibutton(app.RTSSPanel, 'push');
            app.FastRewindButton.ButtonPushedFcn = createCallbackFcn(app, @FastRewindButtonPushed, true);
            app.FastRewindButton.Icon = fullfile(pathToMLAPP, 'FastRewind.png');
            app.FastRewindButton.BackgroundColor = [0.9412 0.9412 0.9412];
            app.FastRewindButton.Enable = 'off';
            app.FastRewindButton.Position = [112 130 47 23];
            app.FastRewindButton.Text = '';

            % Create ReplayButton
            app.ReplayButton = uibutton(app.RTSSPanel, 'push');
            app.ReplayButton.ButtonPushedFcn = createCallbackFcn(app, @ReplayButtonPushed, true);
            app.ReplayButton.BackgroundColor = [0.9412 0.9412 0.9412];
            app.ReplayButton.Enable = 'off';
            app.ReplayButton.Position = [13 130 85 23];
            app.ReplayButton.Text = 'Replay';

            % Create DistanceRemainingValue
            app.DistanceRemainingValue = uilabel(app.RTSSPanel);
            app.DistanceRemainingValue.HorizontalAlignment = 'right';
            app.DistanceRemainingValue.Position = [132 45 53 22];
            app.DistanceRemainingValue.Text = '0.00 m';

            % Create DistanceRemainingLabel
            app.DistanceRemainingLabel = uilabel(app.RTSSPanel);
            app.DistanceRemainingLabel.Position = [18 46 116 22];
            app.DistanceRemainingLabel.Text = 'Distance Remaining:';

            % Create DistanceCoveredValue
            app.DistanceCoveredValue = uilabel(app.RTSSPanel);
            app.DistanceCoveredValue.HorizontalAlignment = 'right';
            app.DistanceCoveredValue.Position = [122 67 63 22];
            app.DistanceCoveredValue.Text = '0.00 m';

            % Create DistanceCoveredLabel
            app.DistanceCoveredLabel = uilabel(app.RTSSPanel);
            app.DistanceCoveredLabel.Position = [18 67 104 22];
            app.DistanceCoveredLabel.Text = 'Distance Covered:';

            % Create CurrentHeightValue
            app.CurrentHeightValue = uilabel(app.RTSSPanel);
            app.CurrentHeightValue.HorizontalAlignment = 'right';
            app.CurrentHeightValue.Position = [122 88 63 22];
            app.CurrentHeightValue.Text = '0.00 m';

            % Create CurrentHeightLabel
            app.CurrentHeightLabel = uilabel(app.RTSSPanel);
            app.CurrentHeightLabel.Position = [18 88 86 22];
            app.CurrentHeightLabel.Text = 'Current Height:';

            % Create TimeRemainingValue
            app.TimeRemainingValue = uilabel(app.RTSSPanel);
            app.TimeRemainingValue.HorizontalAlignment = 'right';
            app.TimeRemainingValue.Position = [380 67 44 22];
            app.TimeRemainingValue.Text = '0.00 s';

            % Create TimeRemainingLabel
            app.TimeRemainingLabel = uilabel(app.RTSSPanel);
            app.TimeRemainingLabel.Position = [285 67 95 22];
            app.TimeRemainingLabel.Text = 'Time Remaining:';

            % Create TimeElapsedValue
            app.TimeElapsedValue = uilabel(app.RTSSPanel);
            app.TimeElapsedValue.HorizontalAlignment = 'right';
            app.TimeElapsedValue.Position = [380 88 44 22];
            app.TimeElapsedValue.Text = '0.00 s';

            % Create TimeElapsedLabel
            app.TimeElapsedLabel = uilabel(app.RTSSPanel);
            app.TimeElapsedLabel.Position = [285 88 85 22];
            app.TimeElapsedLabel.Text = 'Time Elapsed: ';

            % Create LocationTrackerPanel
            app.LocationTrackerPanel = uipanel(app.simlayout);
            app.LocationTrackerPanel.AutoResizeChildren = 'off';
            app.LocationTrackerPanel.Title = 'Location Tracker';
            app.LocationTrackerPanel.BackgroundColor = [0.9412 0.9412 0.9412];
            app.LocationTrackerPanel.Layout.Row = [7 8];
            app.LocationTrackerPanel.Layout.Column = [1 2];
            app.LocationTrackerPanel.FontWeight = 'bold';

            % Create CheckobjectlocationatLabel
            app.CheckobjectlocationatLabel = uilabel(app.LocationTrackerPanel);
            app.CheckobjectlocationatLabel.BackgroundColor = [0.9412 0.9412 0.9412];
            app.CheckobjectlocationatLabel.Position = [12 68 186 21];
            app.CheckobjectlocationatLabel.Text = 'Check object location at';

            % Create locationdisp
            app.locationdisp = uitextarea(app.LocationTrackerPanel);
            app.locationdisp.Editable = 'off';
            app.locationdisp.HorizontalAlignment = 'center';
            app.locationdisp.WordWrap = 'off';
            app.locationdisp.FontSize = 14;
            app.locationdisp.FontWeight = 'bold';
            app.locationdisp.BackgroundColor = [0.9412 0.9412 0.9412];
            app.locationdisp.Placeholder = 'At time t s, the object is at position (x, y)';
            app.locationdisp.Position = [12 13 425 39];

            % Create secLabel
            app.secLabel = uilabel(app.LocationTrackerPanel);
            app.secLabel.HorizontalAlignment = 'center';
            app.secLabel.FontWeight = 'bold';
            app.secLabel.Enable = 'off';
            app.secLabel.Position = [280 68 25 22];
            app.secLabel.Text = 'sec';

            % Create locate
            app.locate = uibutton(app.LocationTrackerPanel, 'push');
            app.locate.ButtonPushedFcn = createCallbackFcn(app, @locateButtonPushed, true);
            app.locate.BackgroundColor = [0.9412 0.9412 0.9412];
            app.locate.FontSize = 14;
            app.locate.FontWeight = 'bold';
            app.locate.Enable = 'off';
            app.locate.Position = [338 67 100 23];
            app.locate.Text = 'Locate';

            % Create timelocate
            app.timelocate = uieditfield(app.LocationTrackerPanel, 'numeric');
            app.timelocate.Editable = 'off';
            app.timelocate.HorizontalAlignment = 'center';
            app.timelocate.BackgroundColor = [0.9412 0.9412 0.9412];
            app.timelocate.Enable = 'off';
            app.timelocate.Position = [157 69 108 20];

            % Create OutputPanel
            app.OutputPanel = uipanel(app.simlayout);
            app.OutputPanel.AutoResizeChildren = 'off';
            app.OutputPanel.Title = 'Output';
            app.OutputPanel.BackgroundColor = [0.9412 0.9412 0.9412];
            app.OutputPanel.Layout.Row = [4 6];
            app.OutputPanel.Layout.Column = [1 2];
            app.OutputPanel.FontWeight = 'bold';

            % Create mLabel_4
            app.mLabel_4 = uilabel(app.OutputPanel);
            app.mLabel_4.HorizontalAlignment = 'center';
            app.mLabel_4.FontWeight = 'bold';
            app.mLabel_4.Position = [412 94 25 22];
            app.mLabel_4.Text = 'm';

            % Create radsLabel
            app.radsLabel = uilabel(app.OutputPanel);
            app.radsLabel.HorizontalAlignment = 'center';
            app.radsLabel.FontWeight = 'bold';
            app.radsLabel.Position = [410 124 30 22];
            app.radsLabel.Text = 'rads';

            % Create mLabel_2
            app.mLabel_2 = uilabel(app.OutputPanel);
            app.mLabel_2.HorizontalAlignment = 'center';
            app.mLabel_2.FontWeight = 'bold';
            app.mLabel_2.Position = [206 64 25 22];
            app.mLabel_2.Text = 'm';

            % Create secLabel_2
            app.secLabel_2 = uilabel(app.OutputPanel);
            app.secLabel_2.HorizontalAlignment = 'center';
            app.secLabel_2.FontWeight = 'bold';
            app.secLabel_2.Position = [206 94 25 22];
            app.secLabel_2.Text = 'sec';

            % Create msLabel
            app.msLabel = uilabel(app.OutputPanel);
            app.msLabel.HorizontalAlignment = 'center';
            app.msLabel.FontWeight = 'bold';
            app.msLabel.Position = [205 124 27 22];
            app.msLabel.Text = 'm/s';

            % Create RangeLabel_2
            app.RangeLabel_2 = uilabel(app.OutputPanel);
            app.RangeLabel_2.BackgroundColor = [0.9412 0.9412 0.9412];
            app.RangeLabel_2.Position = [268 94 43 22];
            app.RangeLabel_2.Text = 'Range:';

            % Create Range
            app.Range = uieditfield(app.OutputPanel, 'text');
            app.Range.Editable = 'off';
            app.Range.HorizontalAlignment = 'center';
            app.Range.BackgroundColor = [0.9412 0.9412 0.9412];
            app.Range.Placeholder = '0';
            app.Range.Position = [348 94 55 22];

            % Create AngleofLaunchLabel
            app.AngleofLaunchLabel = uilabel(app.OutputPanel);
            app.AngleofLaunchLabel.BackgroundColor = [0.9412 0.9412 0.9412];
            app.AngleofLaunchLabel.Position = [268 124 88 22];
            app.AngleofLaunchLabel.Text = 'Angle:';

            % Create angle
            app.angle = uieditfield(app.OutputPanel, 'numeric');
            app.angle.Editable = 'off';
            app.angle.HorizontalAlignment = 'center';
            app.angle.BackgroundColor = [0.9412 0.9412 0.9412];
            app.angle.Position = [348 125 56 22];

            % Create PeakLocationEditFieldLabel
            app.PeakLocationEditFieldLabel = uilabel(app.OutputPanel);
            app.PeakLocationEditFieldLabel.BackgroundColor = [0.9412 0.9412 0.9412];
            app.PeakLocationEditFieldLabel.Position = [12 32 85 22];
            app.PeakLocationEditFieldLabel.Text = 'Peak Location:';

            % Create peak
            app.peak = uieditfield(app.OutputPanel, 'text');
            app.peak.Editable = 'off';
            app.peak.HorizontalAlignment = 'center';
            app.peak.BackgroundColor = [0.9412 0.9412 0.9412];
            app.peak.Placeholder = '(x,y)';
            app.peak.Position = [96 32 100 22];

            % Create MaxHieghtLabel
            app.MaxHieghtLabel = uilabel(app.OutputPanel);
            app.MaxHieghtLabel.BackgroundColor = [0.9412 0.9412 0.9412];
            app.MaxHieghtLabel.Position = [12 64 70 22];
            app.MaxHieghtLabel.Text = 'Max Height:';

            % Create MH
            app.MH = uieditfield(app.OutputPanel, 'numeric');
            app.MH.Editable = 'off';
            app.MH.HorizontalAlignment = 'center';
            app.MH.BackgroundColor = [0.9412 0.9412 0.9412];
            app.MH.Position = [96 64 101 22];

            % Create TimeofflightLabel
            app.TimeofflightLabel = uilabel(app.OutputPanel);
            app.TimeofflightLabel.Position = [12 94 78 22];
            app.TimeofflightLabel.Text = 'Time of flight:';

            % Create timeofflight
            app.timeofflight = uieditfield(app.OutputPanel, 'numeric');
            app.timeofflight.Editable = 'off';
            app.timeofflight.HorizontalAlignment = 'center';
            app.timeofflight.BackgroundColor = [0.9412 0.9412 0.9412];
            app.timeofflight.Position = [96 94 100 22];

            % Create VelocityLabel
            app.VelocityLabel = uilabel(app.OutputPanel);
            app.VelocityLabel.BackgroundColor = [0.9412 0.9412 0.9412];
            app.VelocityLabel.Position = [12 124 53 22];
            app.VelocityLabel.Text = 'Velocity: ';

            % Create velocity
            app.velocity = uieditfield(app.OutputPanel, 'numeric');
            app.velocity.Editable = 'off';
            app.velocity.HorizontalAlignment = 'center';
            app.velocity.BackgroundColor = [0.9412 0.9412 0.9412];
            app.velocity.Position = [96 124 100 22];

            % Create mLabel_5
            app.mLabel_5 = uilabel(app.OutputPanel);
            app.mLabel_5.HorizontalAlignment = 'center';
            app.mLabel_5.FontWeight = 'bold';
            app.mLabel_5.Position = [206 32 25 22];
            app.mLabel_5.Text = 'm';

            % Create TimeatPeakLabel
            app.TimeatPeakLabel = uilabel(app.OutputPanel);
            app.TimeatPeakLabel.BackgroundColor = [0.9412 0.9412 0.9412];
            app.TimeatPeakLabel.Position = [278 62 75 22];
            app.TimeatPeakLabel.Text = 'Time at Peak';

            % Create RangeLabel_3
            app.RangeLabel_3 = uilabel(app.OutputPanel);
            app.RangeLabel_3.BackgroundColor = [0.9412 0.9412 0.9412];
            app.RangeLabel_3.Position = [268 64 79 22];
            app.RangeLabel_3.Text = 'Time at Peak:';

            % Create secLabel_3
            app.secLabel_3 = uilabel(app.OutputPanel);
            app.secLabel_3.HorizontalAlignment = 'center';
            app.secLabel_3.FontWeight = 'bold';
            app.secLabel_3.Position = [412 64 25 22];
            app.secLabel_3.Text = 'sec';

            % Create TimePeak
            app.TimePeak = uieditfield(app.OutputPanel, 'text');
            app.TimePeak.Editable = 'off';
            app.TimePeak.HorizontalAlignment = 'center';
            app.TimePeak.BackgroundColor = [0.9412 0.9412 0.9412];
            app.TimePeak.Placeholder = '0';
            app.TimePeak.Position = [348 64 55 22];

            % Create InputPanel
            app.InputPanel = uipanel(app.simlayout);
            app.InputPanel.AutoResizeChildren = 'off';
            app.InputPanel.Title = 'Input  ';
            app.InputPanel.BackgroundColor = [0.9412 0.9412 0.9412];
            app.InputPanel.Layout.Row = [1 3];
            app.InputPanel.Layout.Column = [1 2];
            app.InputPanel.FontWeight = 'bold';

            % Create AccelerationLabel
            app.AccelerationLabel = uilabel(app.InputPanel);
            app.AccelerationLabel.BackgroundColor = [0.9412 0.9412 0.9412];
            app.AccelerationLabel.Position = [12 65 75 22];
            app.AccelerationLabel.Text = 'Acceleration:';

            % Create IntialHeightLabel
            app.IntialHeightLabel = uilabel(app.InputPanel);
            app.IntialHeightLabel.BackgroundColor = [0.9412 0.9412 0.9412];
            app.IntialHeightLabel.Position = [12 96 108 22];
            app.IntialHeightLabel.Text = 'Height of obstacle: ';

            % Create RangeLabel
            app.RangeLabel = uilabel(app.InputPanel);
            app.RangeLabel.BackgroundColor = [0.9412 0.9412 0.9412];
            app.RangeLabel.Position = [12 129 134 22];
            app.RangeLabel.Text = 'Distance from obstacle: ';

            % Create mLabel
            app.mLabel = uilabel(app.InputPanel);
            app.mLabel.HorizontalAlignment = 'center';
            app.mLabel.FontWeight = 'bold';
            app.mLabel.Position = [219 96 25 22];
            app.mLabel.Text = 'm';

            % Create mLabel_3
            app.mLabel_3 = uilabel(app.InputPanel);
            app.mLabel_3.HorizontalAlignment = 'center';
            app.mLabel_3.FontWeight = 'bold';
            app.mLabel_3.Position = [219 129 25 22];
            app.mLabel_3.Text = 'm';

            % Create RTSSSwitchLabel
            app.RTSSSwitchLabel = uilabel(app.InputPanel);
            app.RTSSSwitchLabel.HorizontalAlignment = 'center';
            app.RTSSSwitchLabel.Tooltip = {'Real Time Simulator System'};
            app.RTSSSwitchLabel.Position = [374 80 35 22];
            app.RTSSSwitchLabel.Text = 'RTSS';

            % Create resetall
            app.resetall = uibutton(app.InputPanel, 'push');
            app.resetall.ButtonPushedFcn = createCallbackFcn(app, @resetallButtonPushed, true);
            app.resetall.BackgroundColor = [0.9412 0.9412 0.9412];
            app.resetall.FontSize = 14;
            app.resetall.FontWeight = 'bold';
            app.resetall.Position = [333 10 100 41];
            app.resetall.Text = 'Reset All';

            % Create SaveButton
            app.SaveButton = uibutton(app.InputPanel, 'push');
            app.SaveButton.ButtonPushedFcn = createCallbackFcn(app, @SaveButtonPushed, true);
            app.SaveButton.WordWrap = 'on';
            app.SaveButton.BackgroundColor = [0.9412 0.9412 0.9412];
            app.SaveButton.FontSize = 14;
            app.SaveButton.FontWeight = 'bold';
            app.SaveButton.Enable = 'off';
            app.SaveButton.Position = [179 9 100 42];
            app.SaveButton.Text = '💾 Save';

            % Create simulate
            app.simulate = uibutton(app.InputPanel, 'push');
            app.simulate.ButtonPushedFcn = createCallbackFcn(app, @simulateButtonPushed, true);
            app.simulate.BackgroundColor = [0.9412 0.9412 0.9412];
            app.simulate.FontSize = 14;
            app.simulate.FontWeight = 'bold';
            app.simulate.Position = [25 10 100 41];
            app.simulate.Text = 'Simulate';

            % Create RTSSSwitch
            app.RTSSSwitch = uiswitch(app.InputPanel, 'slider');
            app.RTSSSwitch.Position = [370 117 41 18];

            % Create acceleration
            app.acceleration = uidropdown(app.InputPanel);
            app.acceleration.Items = {'Earth', 'Moon', 'Mars'};
            app.acceleration.BackgroundColor = [0.9412 0.9412 0.9412];
            app.acceleration.Position = [143 65 73 22];
            app.acceleration.Value = 'Earth';

            % Create Height
            app.Height = uieditfield(app.InputPanel, 'numeric');
            app.Height.HorizontalAlignment = 'center';
            app.Height.BackgroundColor = [0.9412 0.9412 0.9412];
            app.Height.Position = [143 96 73 22];

            % Create Distance
            app.Distance = uieditfield(app.InputPanel, 'numeric');
            app.Distance.HorizontalAlignment = 'center';
            app.Distance.BackgroundColor = [0.9412 0.9412 0.9412];
            app.Distance.Position = [143 129 73 22];

            % Create GraphsTab
            app.GraphsTab = uitab(app.TabGroup);
            app.GraphsTab.AutoResizeChildren = 'off';
            app.GraphsTab.Title = 'Graphs';
            app.GraphsTab.BackgroundColor = [0.9412 0.9412 0.9412];
            app.GraphsTab.ButtonDownFcn = createCallbackFcn(app, @GraphsTabButtonDown, true);

            % Create GridLayout
            app.GridLayout = uigridlayout(app.GraphsTab);
            app.GridLayout.ColumnWidth = {'1x', '1x', '1x', '1x', '1x', '1x'};
            app.GridLayout.RowHeight = {'1x', '1x', '1x', '1x', '1x', '1x', '1x', '1x', '1x'};
            app.GridLayout.BackgroundColor = [0.9412 0.9412 0.9412];

            % Create AccelerationTimeGraph
            app.AccelerationTimeGraph = uiaxes(app.GridLayout);
            title(app.AccelerationTimeGraph, 'Acceleration - Time')
            xlabel(app.AccelerationTimeGraph, 'Time (s)')
            ylabel(app.AccelerationTimeGraph, 'Acceleration (m/s²)')
            zlabel(app.AccelerationTimeGraph, 'Z')
            app.AccelerationTimeGraph.XLimitMethod = 'tight';
            app.AccelerationTimeGraph.YLimitMethod = 'tight';
            app.AccelerationTimeGraph.ZLimitMethod = 'tight';
            app.AccelerationTimeGraph.MinorGridLineStyle = '-';
            app.AccelerationTimeGraph.Color = 'none';
            app.AccelerationTimeGraph.XGrid = 'on';
            app.AccelerationTimeGraph.XMinorGrid = 'on';
            app.AccelerationTimeGraph.YGrid = 'on';
            app.AccelerationTimeGraph.YMinorGrid = 'on';
            app.AccelerationTimeGraph.ColorOrder = [1 0.0705882352941176 0.650980392156863;0.96078431372549 0.466666666666667 0.16078431372549;1 0.909803921568627 0.392156862745098;0.752941176470588 0.36078431372549 0.984313725490196;0.286274509803922 0.858823529411765 0.250980392156863;0.423529411764706 0.956862745098039 1;1 0.0705882352941176 0.650980392156863];
            app.AccelerationTimeGraph.Layout.Row = [1 6];
            app.AccelerationTimeGraph.Layout.Column = [1 2];
            colormap(app.AccelerationTimeGraph, 'copper')

            % Create VelocityTimeGraph
            app.VelocityTimeGraph = uiaxes(app.GridLayout);
            title(app.VelocityTimeGraph, 'Velocity - Time')
            xlabel(app.VelocityTimeGraph, 'Time (s)')
            ylabel(app.VelocityTimeGraph, 'Velocity (m/s)')
            zlabel(app.VelocityTimeGraph, 'Z')
            app.VelocityTimeGraph.Toolbar.Visible = 'off';
            app.VelocityTimeGraph.XLimitMethod = 'tight';
            app.VelocityTimeGraph.YLimitMethod = 'tight';
            app.VelocityTimeGraph.ZLimitMethod = 'tight';
            app.VelocityTimeGraph.MinorGridLineStyle = '-';
            app.VelocityTimeGraph.Color = 'none';
            app.VelocityTimeGraph.XGrid = 'on';
            app.VelocityTimeGraph.XMinorGrid = 'on';
            app.VelocityTimeGraph.YGrid = 'on';
            app.VelocityTimeGraph.YMinorGrid = 'on';
            app.VelocityTimeGraph.ColorOrder = [0.290196078431373 0.858823529411765 0.250980392156863;0.96078431372549 0.466666666666667 0.16078431372549;1 0.909803921568627 0.388235294117647;0.749019607843137 0.36078431372549 0.980392156862745;0.290196078431373 0.858823529411765 0.250980392156863;0.423529411764706 0.956862745098039 1;0.949019607843137 0.403921568627451 0.772549019607843];
            app.VelocityTimeGraph.Layout.Row = [1 6];
            app.VelocityTimeGraph.Layout.Column = [3 4];

            % Create DisplacementTimeGraph
            app.DisplacementTimeGraph = uiaxes(app.GridLayout);
            title(app.DisplacementTimeGraph, 'Displacement - Time')
            xlabel(app.DisplacementTimeGraph, 'Time (s)')
            ylabel(app.DisplacementTimeGraph, 'Displacement (m)')
            zlabel(app.DisplacementTimeGraph, 'Z')
            app.DisplacementTimeGraph.Toolbar.Visible = 'off';
            app.DisplacementTimeGraph.XLimitMethod = 'tight';
            app.DisplacementTimeGraph.YLimitMethod = 'tight';
            app.DisplacementTimeGraph.ZLimitMethod = 'tight';
            app.DisplacementTimeGraph.MinorGridLineStyle = '-';
            app.DisplacementTimeGraph.Color = 'none';
            app.DisplacementTimeGraph.XGrid = 'on';
            app.DisplacementTimeGraph.XMinorGrid = 'on';
            app.DisplacementTimeGraph.YGrid = 'on';
            app.DisplacementTimeGraph.YMinorGrid = 'on';
            app.DisplacementTimeGraph.ColorOrder = [0.149 0.549 0.866;0.96 0.466 0.16;1 0.909 0.392;0.752 0.36 0.984;0.286 0.858 0.25;0.423 0.956 1;0.949 0.403 0.772];
            app.DisplacementTimeGraph.Layout.Row = [1 6];
            app.DisplacementTimeGraph.Layout.Column = [5 6];

            % Create MaxDisplacementLabel
            app.MaxDisplacementLabel = uilabel(app.GridLayout);
            app.MaxDisplacementLabel.BackgroundColor = [0.9412 0.9412 0.9412];
            app.MaxDisplacementLabel.HorizontalAlignment = 'center';
            app.MaxDisplacementLabel.Layout.Row = 7;
            app.MaxDisplacementLabel.Layout.Column = 5;
            app.MaxDisplacementLabel.Text = 'Maximum Displacement';

            % Create MaxDisplacement
            app.MaxDisplacement = uieditfield(app.GridLayout, 'numeric');
            app.MaxDisplacement.Editable = 'off';
            app.MaxDisplacement.HorizontalAlignment = 'center';
            app.MaxDisplacement.BackgroundColor = [0.9412 0.9412 0.9412];
            app.MaxDisplacement.Layout.Row = 7;
            app.MaxDisplacement.Layout.Column = 6;

            % Create MaxDisplacementLabel_2
            app.MaxDisplacementLabel_2 = uilabel(app.GridLayout);
            app.MaxDisplacementLabel_2.BackgroundColor = [0.9412 0.9412 0.9412];
            app.MaxDisplacementLabel_2.HorizontalAlignment = 'center';
            app.MaxDisplacementLabel_2.Layout.Row = 8;
            app.MaxDisplacementLabel_2.Layout.Column = 5;
            app.MaxDisplacementLabel_2.Text = 'Minimum Displacement';

            % Create MinDisplacement
            app.MinDisplacement = uieditfield(app.GridLayout, 'numeric');
            app.MinDisplacement.Editable = 'off';
            app.MinDisplacement.HorizontalAlignment = 'center';
            app.MinDisplacement.BackgroundColor = [0.9412 0.9412 0.9412];
            app.MinDisplacement.Layout.Row = 8;
            app.MinDisplacement.Layout.Column = 6;

            % Create MaxDisplacementLabel_3
            app.MaxDisplacementLabel_3 = uilabel(app.GridLayout);
            app.MaxDisplacementLabel_3.BackgroundColor = [0.9412 0.9412 0.9412];
            app.MaxDisplacementLabel_3.HorizontalAlignment = 'center';
            app.MaxDisplacementLabel_3.Layout.Row = 9;
            app.MaxDisplacementLabel_3.Layout.Column = 5;
            app.MaxDisplacementLabel_3.Text = 'Average Velocity (m/s)';

            % Create AvgVelocity
            app.AvgVelocity = uieditfield(app.GridLayout, 'numeric');
            app.AvgVelocity.Editable = 'off';
            app.AvgVelocity.HorizontalAlignment = 'center';
            app.AvgVelocity.BackgroundColor = [0.9412 0.9412 0.9412];
            app.AvgVelocity.Layout.Row = 9;
            app.AvgVelocity.Layout.Column = 6;

            % Create MaxDisplacementLabel_4
            app.MaxDisplacementLabel_4 = uilabel(app.GridLayout);
            app.MaxDisplacementLabel_4.BackgroundColor = [0.9412 0.9412 0.9412];
            app.MaxDisplacementLabel_4.HorizontalAlignment = 'center';
            app.MaxDisplacementLabel_4.Layout.Row = 7;
            app.MaxDisplacementLabel_4.Layout.Column = 3;
            app.MaxDisplacementLabel_4.Text = 'Maximum Velocity';

            % Create MaxVelocity
            app.MaxVelocity = uieditfield(app.GridLayout, 'numeric');
            app.MaxVelocity.Editable = 'off';
            app.MaxVelocity.HorizontalAlignment = 'center';
            app.MaxVelocity.BackgroundColor = [0.9412 0.9412 0.9412];
            app.MaxVelocity.Layout.Row = 7;
            app.MaxVelocity.Layout.Column = 4;

            % Create MaxDisplacementLabel_5
            app.MaxDisplacementLabel_5 = uilabel(app.GridLayout);
            app.MaxDisplacementLabel_5.BackgroundColor = [0.9412 0.9412 0.9412];
            app.MaxDisplacementLabel_5.HorizontalAlignment = 'center';
            app.MaxDisplacementLabel_5.Layout.Row = 8;
            app.MaxDisplacementLabel_5.Layout.Column = 3;
            app.MaxDisplacementLabel_5.Text = 'Minimum Velocity';

            % Create MinVelocity
            app.MinVelocity = uieditfield(app.GridLayout, 'numeric');
            app.MinVelocity.Editable = 'off';
            app.MinVelocity.HorizontalAlignment = 'center';
            app.MinVelocity.BackgroundColor = [0.9412 0.9412 0.9412];
            app.MinVelocity.Layout.Row = 8;
            app.MinVelocity.Layout.Column = 4;

            % Create MaxDisplacementLabel_7
            app.MaxDisplacementLabel_7 = uilabel(app.GridLayout);
            app.MaxDisplacementLabel_7.BackgroundColor = [0.9412 0.9412 0.9412];
            app.MaxDisplacementLabel_7.HorizontalAlignment = 'center';
            app.MaxDisplacementLabel_7.Layout.Row = 9;
            app.MaxDisplacementLabel_7.Layout.Column = 3;
            app.MaxDisplacementLabel_7.Text = 'Avg Acceleration (m/s²)';

            % Create AvgAcceleration
            app.AvgAcceleration = uieditfield(app.GridLayout, 'numeric');
            app.AvgAcceleration.Editable = 'off';
            app.AvgAcceleration.HorizontalAlignment = 'center';
            app.AvgAcceleration.BackgroundColor = [0.9412 0.9412 0.9412];
            app.AvgAcceleration.Layout.Row = 9;
            app.AvgAcceleration.Layout.Column = 4;

            % Create MaxDisplacementLabel_8
            app.MaxDisplacementLabel_8 = uilabel(app.GridLayout);
            app.MaxDisplacementLabel_8.BackgroundColor = [0.9412 0.9412 0.9412];
            app.MaxDisplacementLabel_8.HorizontalAlignment = 'center';
            app.MaxDisplacementLabel_8.Layout.Row = 7;
            app.MaxDisplacementLabel_8.Layout.Column = 1;
            app.MaxDisplacementLabel_8.Text = 'Maximum Acceleration';

            % Create MaxAcceleration
            app.MaxAcceleration = uieditfield(app.GridLayout, 'numeric');
            app.MaxAcceleration.Editable = 'off';
            app.MaxAcceleration.HorizontalAlignment = 'center';
            app.MaxAcceleration.BackgroundColor = [0.9412 0.9412 0.9412];
            app.MaxAcceleration.Layout.Row = 7;
            app.MaxAcceleration.Layout.Column = 2;

            % Create MaxDisplacementLabel_9
            app.MaxDisplacementLabel_9 = uilabel(app.GridLayout);
            app.MaxDisplacementLabel_9.BackgroundColor = [0.9412 0.9412 0.9412];
            app.MaxDisplacementLabel_9.HorizontalAlignment = 'center';
            app.MaxDisplacementLabel_9.Layout.Row = 8;
            app.MaxDisplacementLabel_9.Layout.Column = 1;
            app.MaxDisplacementLabel_9.Text = 'Minimum Acceleration';

            % Create MinAcceleration
            app.MinAcceleration = uieditfield(app.GridLayout, 'numeric');
            app.MinAcceleration.Editable = 'off';
            app.MinAcceleration.HorizontalAlignment = 'center';
            app.MinAcceleration.BackgroundColor = [0.9412 0.9412 0.9412];
            app.MinAcceleration.Layout.Row = 8;
            app.MinAcceleration.Layout.Column = 2;

            % Create MaxDisplacementLabel_10
            app.MaxDisplacementLabel_10 = uilabel(app.GridLayout);
            app.MaxDisplacementLabel_10.BackgroundColor = [0.9412 0.9412 0.9412];
            app.MaxDisplacementLabel_10.HorizontalAlignment = 'center';
            app.MaxDisplacementLabel_10.Layout.Row = 9;
            app.MaxDisplacementLabel_10.Layout.Column = 1;
            app.MaxDisplacementLabel_10.Text = 'Jerk (m/s³)';

            % Create Jerk
            app.Jerk = uieditfield(app.GridLayout, 'numeric');
            app.Jerk.Editable = 'off';
            app.Jerk.HorizontalAlignment = 'center';
            app.Jerk.BackgroundColor = [0.9412 0.9412 0.9412];
            app.Jerk.Layout.Row = 9;
            app.Jerk.Layout.Column = 2;

            % Create HistoryTab
            app.HistoryTab = uitab(app.TabGroup);
            app.HistoryTab.AutoResizeChildren = 'off';
            app.HistoryTab.Title = 'History';
            app.HistoryTab.BackgroundColor = [0.9412 0.9412 0.9412];
            app.HistoryTab.ButtonDownFcn = createCallbackFcn(app, @HistoryTabButtonDown, true);

            % Create GridLayout2
            app.GridLayout2 = uigridlayout(app.HistoryTab);
            app.GridLayout2.ColumnWidth = {'1x', '1x', '1x', '1x', '1x'};
            app.GridLayout2.RowHeight = {'1x', '1x', '1x', '1x', '1x', '1x', '1x', '1x', '1x'};
            app.GridLayout2.BackgroundColor = [0.9412 0.9412 0.9412];

            % Create Historygraph
            app.Historygraph = uiaxes(app.GridLayout2);
            title(app.Historygraph, 'History Graph')
            xlabel(app.Historygraph, 'Horizontal Distance (m)')
            ylabel(app.Historygraph, 'Vertical Distance (m)')
            zlabel(app.Historygraph, 'Z')
            app.Historygraph.Color = 'none';
            app.Historygraph.XGrid = 'on';
            app.Historygraph.YGrid = 'on';
            app.Historygraph.ColorOrder = [0.149 0.549 0.866;0.96 0.466 0.16;1 0.909 0.392;0.752 0.36 0.984;0.286 0.858 0.25;0.423 0.956 1;0.949 0.403 0.772];
            app.Historygraph.Layout.Row = [4 9];
            app.Historygraph.Layout.Column = [1 3];

            % Create SimulationH
            app.SimulationH = uitable(app.GridLayout2);
            app.SimulationH.BackgroundColor = [1 1 1;0.9412 0.9412 0.9412];
            app.SimulationH.ColumnName = {'Name'; 'Distance'; 'Height'; 'Acceleration'; 'Velocity'; 'Angle of Launch (rads)'; 'Time of Flight'};
            app.SimulationH.RowName = {};
            app.SimulationH.Layout.Row = [1 3];
            app.SimulationH.Layout.Column = [1 5];

            % Create TabGroup2
            app.TabGroup2 = uitabgroup(app.GridLayout2);
            app.TabGroup2.AutoResizeChildren = 'off';
            app.TabGroup2.Layout.Row = [4 9];
            app.TabGroup2.Layout.Column = [4 5];

            % Create MainTab
            app.MainTab = uitab(app.TabGroup2);
            app.MainTab.AutoResizeChildren = 'off';
            app.MainTab.Title = 'Main';
            app.MainTab.BackgroundColor = 'none';

            % Create ImportButton
            app.ImportButton = uibutton(app.MainTab, 'push');
            app.ImportButton.ButtonPushedFcn = createCallbackFcn(app, @ImportButtonPushed, true);
            app.ImportButton.BackgroundColor = [0.9412 0.9412 0.9412];
            app.ImportButton.Position = [86 138 100 23];
            app.ImportButton.Text = 'Import';

            % Create DataDropDownLabel
            app.DataDropDownLabel = uilabel(app.MainTab);
            app.DataDropDownLabel.BackgroundColor = [0.9412 0.9412 0.9412];
            app.DataDropDownLabel.Position = [104 181 30 22];
            app.DataDropDownLabel.Text = 'Data';

            % Create DataDropDown
            app.DataDropDown = uidropdown(app.MainTab);
            app.DataDropDown.Items = {''};
            app.DataDropDown.BackgroundColor = [0.9412 0.9412 0.9412];
            app.DataDropDown.Position = [185 181 100 22];
            app.DataDropDown.Value = '';

            % Create ClearAllButton
            app.ClearAllButton = uibutton(app.MainTab, 'push');
            app.ClearAllButton.ButtonPushedFcn = createCallbackFcn(app, @ClearAllButtonPushed, true);
            app.ClearAllButton.BackgroundColor = [0.9412 0.9412 0.9412];
            app.ClearAllButton.Position = [203 138 100 23];
            app.ClearAllButton.Text = 'Clear All';

            % Create ProjectileSimulatorLabel
            app.ProjectileSimulatorLabel = uilabel(app.UIFigure);
            app.ProjectileSimulatorLabel.HorizontalAlignment = 'center';
            app.ProjectileSimulatorLabel.FontSize = 36;
            app.ProjectileSimulatorLabel.FontWeight = 'bold';
            app.ProjectileSimulatorLabel.Position = [307 557 348 48];
            app.ProjectileSimulatorLabel.Text = 'Projectile Simulator';

            % Create Versionlabel
            app.Versionlabel = uilabel(app.UIFigure);
            app.Versionlabel.HorizontalAlignment = 'center';
            app.Versionlabel.FontWeight = 'bold';
            app.Versionlabel.Position = [923 570 25 22];
            app.Versionlabel.Text = 'V4';

            % Show the figure after all components are created
            app.UIFigure.Visible = 'on';
        end
    end

    % App creation and deletion
    methods (Access = public)

        % Construct app
        function app = Projectile_Simulator_TP_exported

            runningApp = getRunningApp(app);

            % Check for running singleton app
            if isempty(runningApp)

                % Create UIFigure and components
                createComponents(app)

                % Register the app with App Designer
                registerApp(app, app.UIFigure)
            else

                % Focus the running singleton app
                figure(runningApp.UIFigure)

                app = runningApp;
            end

            if nargout == 0
                clear app
            end
        end

        % Code that executes before app deletion
        function delete(app)

            % Delete UIFigure when app is deleted
            delete(app.UIFigure)
        end
    end
end