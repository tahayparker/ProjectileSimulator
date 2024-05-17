classdef Projectile_Simulator_TP_exported < matlab.apps.AppBase

    % Properties that correspond to app components
    properties (Access = public)
        UIFigure                    matlab.ui.Figure
        Versionlabel                matlab.ui.control.Label
        ProjectileSimulatorLabel    matlab.ui.control.Label
        TabGroup                    matlab.ui.container.TabGroup
        simulator                   matlab.ui.container.Tab
        simlayout                   matlab.ui.container.GridLayout
        LocationTrackerPanel        matlab.ui.container.Panel
        locate                      matlab.ui.control.Button
        secLabel                    matlab.ui.control.Label
        locationdisp                matlab.ui.control.TextArea
        timelocate                  matlab.ui.control.NumericEditField
        EnterthetimetocheckthelocationoftheobjectEditFieldLabel  matlab.ui.control.Label
        OutputPanel                 matlab.ui.container.Panel
        mLabel_4                    matlab.ui.control.Label
        mLabel_3                    matlab.ui.control.Label
        mLabel_2                    matlab.ui.control.Label
        secLabel_2                  matlab.ui.control.Label
        MH                          matlab.ui.control.NumericEditField
        MaxHieghtLabel              matlab.ui.control.Label
        peak                        matlab.ui.control.EditField
        PeakLocationEditFieldLabel  matlab.ui.control.Label
        range                       matlab.ui.control.NumericEditField
        RangeLabel                  matlab.ui.control.Label
        timeofflight                matlab.ui.control.NumericEditField
        TimeofflightLabel           matlab.ui.control.Label
        InputPanel                  matlab.ui.container.Panel
        acceleration                matlab.ui.control.DropDown
        AccelerationLabel           matlab.ui.control.Label
        degLabel                    matlab.ui.control.Label
        mLabel                      matlab.ui.control.Label
        msLabel                     matlab.ui.control.Label
        RTSSSwitch                  matlab.ui.control.Switch
        RTSSSwitchLabel             matlab.ui.control.Label
        resetall                    matlab.ui.control.Button
        simulate                    matlab.ui.control.Button
        Hieght                      matlab.ui.control.NumericEditField
        IntialHeightLabel           matlab.ui.control.Label
        angle                       matlab.ui.control.NumericEditField
        AngleofLaunchLabel          matlab.ui.control.Label
        velocity                    matlab.ui.control.NumericEditField
        VelocityLabel               matlab.ui.control.Label
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
        RewindButton                matlab.ui.control.Button
        PlayPauseButton             matlab.ui.control.StateButton
        ForwardButton               matlab.ui.control.Button
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
        AccelerationTimeGraph       matlab.ui.control.UIAxes
        VelocityTimeGraph           matlab.ui.control.UIAxes
        DisplacementTimeGraph       matlab.ui.control.UIAxes
        HistoryTab                  matlab.ui.container.Tab
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
        timerr % Timer

        playIcon = 'Play.png' % Play Icon
        pauseIcon = 'Pause.png' % Pause Icon
    end

    methods (Access = private)

        function [V,alpha,g,h] = input(app) % This is the input function of the program
            V = app.velocity.Value; % Velocity
            h = app.Hieght.Value;
            alpha =  deg2rad(app.angle.Value); % Angle in radians
            a = app.acceleration.Value; % Acceleration ie. Gravity
            if a == "Earth"
                g = 9.81; % Earth Gravity
            elseif a == "Moon"
                g = 1.62; % Moon Gravity
            elseif a == "Mars"
                g = 3.71; % Mars Gravity
            end
        end

        function [tofflight, max_distance] = sim(app) % runs the simulation function
            [V,alpha,g,h] = input(app);
            vy = V * sin(alpha);
            vx = V * cos(alpha);

            tofflight = (2*V*sin(alpha)/g); % Calculates the time of flight
            app.timeofflight.Value = tofflight;
            app.TMAX = tofflight;
            max_distance = V * cos(alpha) * tofflight; % Calculates the Range
            app.range.Value = max_distance;

            t_max_height = V * sin(alpha) / g;
            y_max_height = V * sin(alpha) * t_max_height - 0.5 * g * t_max_height^2; % Max Hieght Y - Coordinates
            x_max_height = V * cos(alpha) * t_max_height - 0.5 * g * t_max_height^2; % Max Hieght X - Coordinates
            app.peak.Value = ['(',num2str(x_max_height),',',num2str(y_max_height),')'];
            app.MH.Value = y_max_height; % Displays Max Hieght

            app.locate.Enable ="on"; % Enables the locate button
            app.timelocate.Editable = "on"; % Enables the Time field
            time_interval = linspace (0, tofflight, 1000);
            x = vx * time_interval;
            y = h + (vy * time_interval) - (0.5 * g * time_interval.^2);
            y(y < 0) = 0;
            app.X = x;
            app.Y = y;
            app.T = time_interval;
            app.TMAX = tofflight;

            startRTSS(app);

        end

        function plotgraph(app) % Plots the static Graph

            [V,alpha,g] = input(app);
            t = linspace(0, 2 * V * sin(alpha) / g, 100); % Time array
            x = V * cos(alpha) * t; % X - Axis
            y = V * sin(alpha) * t - 0.5 * g * t.^2; % Y - Axis

            plot(app.MainGraph,x,y)
        end

        function locatew(app) % The function locates the projectile in x,y coordinaties in a given time period
            [tofflight] = sim(app);
            [V,alpha,g] = input(app);
            t_check = app.timelocate.Value; % Time field
            x_check = V * cos(alpha) * t_check; % X - Location
            y_check = V * sin(alpha) * t_check - 0.5 * g * t_check^2; % Y - Location

            if t_check >= tofflight % if input time is greater or equal to time of flight it corrects the dislay coordnates
                x_check = V * cos(alpha) * tofflight;
                app.locationdisp.Value= ['At time ' num2str(t_check) ' s, the object is at position (' num2str(x_check) ', 0)'];  % Displays time, (x,y) coordinates in locationdisp if time input is greater or equal to time of flight
            else
                app.locationdisp.Value= ['At time ' num2str(t_check) ' s, the object is at position (' num2str(x_check) ', ' num2str(y_check) ')'];  % Displays time, (x,y) coordinates in locationdisp
            end

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
            else
                plot(app.MainGraph, app.X(end), app.Y(end), 'o', 'MarkerFaceColor', 'red');
                hold(app.MainGraph, 'on');
                plot(app.MainGraph, app.X, app.Y, '-');

                xlim(app.MainGraph, [min(app.X), max(app.X)]);
                ylim(app.MainGraph, [min(app.Y), max(app.Y)]);
                hold(app.MainGraph, 'off');
            end
        end

        function pauseRTSS(app) % This function pauses the animation
            app.isPlay = false;
            app.isUpdat = false;
            stop(app.timerr); % this stops the app timer
            app.PlayPauseButton.Icon = app.playIcon;
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
            app.timerr = timer('ExecutionMode', 'fixedRate', ...
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

        function reset(app) % This function resets the entire program and disables the RTSS panel and Location panel
            app.velocity.Value = 0;
            app.angle.Value = 0;
            sim(app);
            plotgraph(app);
            app.RTSSSwitch.Value = "Off";
            app.peak.Value = "(x,y)";
            app.locate.Enable ="off";
            app.timelocate.Editable = "off";
            app.RTSSPanel.Enable = "off";
            app.ReplayButton.Enable = "off";
            app.FastForwardButton.Enable = "off";
            app.FastRewindButton.Enable= "off";
            app.ReplayButton.Enable= "off";
            app.RewindButton.Enable= "off";
            app.PlayPauseButton.Enable = "off";
            app.ForwardButton.Enable= "off";
            app.InputPlaybackSpeed.Enable= "off";

        end

        function rtssenable(app) % This function enables or disables the RTSS
            rs = app.RTSSSwitch.Value;
            if strcmp(rs,'On') % Check the RTSS switch if it is ON and enables the RTSS panel
                app.RTSSPanel.Enable = "on";
                app.ReplayButton.Enable = "on";
                app.FastForwardButton.Enable = "on";
                app.FastRewindButton.Enable= "on";
                app.ReplayButton.Enable= "on";
                app.RewindButton.Enable= "on";
                app.PlayPauseButton.Enable = "on";
                app.ForwardButton.Enable= "on";
                app.InputPlaybackSpeed.Enable= "on";
                app.sAni = true;
            else % If off then disables the panel along with its buttons
                app.sAni = false;
                app.plotgraph;
                app.RTSSPanel.Enable = "off";
                app.ReplayButton.Enable = "off";
                app.FastForwardButton.Enable = "off";
                app.FastRewindButton.Enable= "off";
                app.ReplayButton.Enable= "off";
                app.RewindButton.Enable= "off";
                app.PlayPauseButton.Enable = "off";
                app.ForwardButton.Enable= "off";
                app.InputPlaybackSpeed.Enable= "off";

            end
        end

        function plotdisp(app)
            [V, alpha, g, h] = input(app);
            max_x = 2 * V * sin(alpha) / g;
            x = linspace(0, max_x, 10000); % Time array
            y = h + V * sin(alpha) * x - 0.5 * g * x.^2; % Displacement based on time
            plot(app.DisplacementTimeGraph, x, y) % Plot y vs x (displacement vs time)
            app.MaxDisplacement.Value = max(y);
            app.AvgVelocity.Value = V * cos(alpha);
        end

        function plotvel(app)
            [V, alpha, g] = input(app);
            disp(V)
            x = linspace(0, 2 * V * sin(alpha) / g, 10000); % Time array
            y = linspace(0, V, 10000);
            plot(app.VelocityTimeGraph, x, y)
            app.MaxVelocity.Value = V;
            app.AvgAcceleration.Value = g;
        end

        function plotacc(app)
            [V, alpha, g] = input(app);
            x = linspace(0, 2 * V * sin(alpha) / g, 10000); % Time array - y axis
            y = linspace(g, g, 10000);
            plot(app.AccelerationTimeGraph, x, y)
            app.MaxAcceleration.Value = g;
            app.Jerk.Value = 0;
        end
    end

    % Callbacks that handle component events
    methods (Access = private)

        % Button pushed function: simulate
        function simulateButtonPushed(app, event)
            input(app); % Input Function
            rtssenable(app); % RTSS Enable
            sim(app); % Simulation
            plotdisp(app);
            plotvel(app);
            plotacc(app);
        end

        % Button pushed function: resetall
        function resetallButtonPushed(app, event)
            reset(app); % Reset Function
        end

        % Button pushed function: locate
        function locateButtonPushed(app, event)
            locatew(app); % Locate function
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
            tweakRTSS(app, -4)
        end

        % Button pushed function: RewindButton
        function RewindButtonPushed(app, event)
            tweakRTSS(app, -2);
        end

        % Value changed function: PlayPauseButton
        function PlayPauseButtonValueChanged(app, event)
            if app.isAni
                if app.isPlay
                    pauseRTSS(app);
                else
                    resumeRTSS(app);
                end
            end

        end

        % Button pushed function: ForwardButton
        function ForwardButtonPushed(app, event)
            tweakRTSS(app, 4);

        end

        % Button pushed function: FastForwardButton
        function FastForwardButtonPushed(app, event)
            tweakRTSS(app, 2);
        end

        % Value changed function: InputPlaybackSpeed
        function InputPlaybackSpeedValueChanged(app, event)
            app.pSpeed = app.InputPlaybackSpeed.Value;

        end
    end

    % Component initialization
    methods (Access = private)

        % Create UIFigure and components
        function createComponents(app)

            % Create UIFigure and hide until all components are created
            app.UIFigure = uifigure('Visible', 'off');
            app.UIFigure.Position = [100 100 957 609];
            app.UIFigure.Name = 'MATLAB App';

            % Create TabGroup
            app.TabGroup = uitabgroup(app.UIFigure);
            app.TabGroup.Position = [1 1 957 550];

            % Create simulator
            app.simulator = uitab(app.TabGroup);
            app.simulator.Title = 'Simulator';

            % Create simlayout
            app.simlayout = uigridlayout(app.simulator);
            app.simlayout.ColumnWidth = {'1.5x', '1.5x', '1x', '1x', '1x'};
            app.simlayout.RowHeight = {'1x', '1x', '1x', '1x', '1x', '1x', '1x', '1x'};

            % Create MainGraph
            app.MainGraph = uiaxes(app.simlayout);
            title(app.MainGraph, 'Projectile Motion')
            xlabel(app.MainGraph, 'Horizontal Distance (m)')
            ylabel(app.MainGraph, 'Vertical Distance (m)')
            zlabel(app.MainGraph, 'Z')
            app.MainGraph.FontSize = 10;
            app.MainGraph.Layout.Row = [1 5];
            app.MainGraph.Layout.Column = [3 5];

            % Create RTSSPanel
            app.RTSSPanel = uipanel(app.simlayout);
            app.RTSSPanel.Enable = 'off';
            app.RTSSPanel.Title = 'Real Time Simulator System ( RTSS )';
            app.RTSSPanel.Layout.Row = [6 8];
            app.RTSSPanel.Layout.Column = [3 5];
            app.RTSSPanel.FontWeight = 'bold';

            % Create InputPlaybackSpeed
            app.InputPlaybackSpeed = uispinner(app.RTSSPanel);
            app.InputPlaybackSpeed.Step = 0.25;
            app.InputPlaybackSpeed.Limits = [0.25 2];
            app.InputPlaybackSpeed.ValueChangedFcn = createCallbackFcn(app, @InputPlaybackSpeedValueChanged, true);
            app.InputPlaybackSpeed.HorizontalAlignment = 'center';
            app.InputPlaybackSpeed.Enable = 'off';
            app.InputPlaybackSpeed.Tooltip = {'Set playback speed'};
            app.InputPlaybackSpeed.Position = [399 130 58 22];
            app.InputPlaybackSpeed.Value = 1;

            % Create FastForwardButton
            app.FastForwardButton = uibutton(app.RTSSPanel, 'push');
            app.FastForwardButton.ButtonPushedFcn = createCallbackFcn(app, @FastForwardButtonPushed, true);
            app.FastForwardButton.Icon = 'fast-forward.png';
            app.FastForwardButton.Enable = 'off';
            app.FastForwardButton.Position = [332 130 47 23];
            app.FastForwardButton.Text = '';

            % Create ForwardButton
            app.ForwardButton = uibutton(app.RTSSPanel, 'push');
            app.ForwardButton.ButtonPushedFcn = createCallbackFcn(app, @ForwardButtonPushed, true);
            app.ForwardButton.Icon = 'Forward.png';
            app.ForwardButton.IconAlignment = 'center';
            app.ForwardButton.Enable = 'off';
            app.ForwardButton.Position = [270 130 47 23];
            app.ForwardButton.Text = '';

            % Create PlayPauseButton
            app.PlayPauseButton = uibutton(app.RTSSPanel, 'state');
            app.PlayPauseButton.ValueChangedFcn = createCallbackFcn(app, @PlayPauseButtonValueChanged, true);
            app.PlayPauseButton.Enable = 'off';
            app.PlayPauseButton.Icon = 'Play.png';
            app.PlayPauseButton.IconAlignment = 'center';
            app.PlayPauseButton.Text = '';
            app.PlayPauseButton.Position = [233 130 25 23];

            % Create RewindButton
            app.RewindButton = uibutton(app.RTSSPanel, 'push');
            app.RewindButton.ButtonPushedFcn = createCallbackFcn(app, @RewindButtonPushed, true);
            app.RewindButton.Icon = 'Rewind.png';
            app.RewindButton.Enable = 'off';
            app.RewindButton.Position = [172 130 47 23];
            app.RewindButton.Text = '';

            % Create FastRewindButton
            app.FastRewindButton = uibutton(app.RTSSPanel, 'push');
            app.FastRewindButton.ButtonPushedFcn = createCallbackFcn(app, @FastRewindButtonPushed, true);
            app.FastRewindButton.Icon = 'FastRewind.png';
            app.FastRewindButton.Enable = 'off';
            app.FastRewindButton.Position = [112 130 47 23];
            app.FastRewindButton.Text = '';

            % Create ReplayButton
            app.ReplayButton = uibutton(app.RTSSPanel, 'push');
            app.ReplayButton.ButtonPushedFcn = createCallbackFcn(app, @ReplayButtonPushed, true);
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

            % Create InputPanel
            app.InputPanel = uipanel(app.simlayout);
            app.InputPanel.Title = 'Input  ';
            app.InputPanel.Layout.Row = [1 3];
            app.InputPanel.Layout.Column = [1 2];
            app.InputPanel.FontWeight = 'bold';

            % Create VelocityLabel
            app.VelocityLabel = uilabel(app.InputPanel);
            app.VelocityLabel.Position = [12 129 53 22];
            app.VelocityLabel.Text = 'Velocity: ';

            % Create velocity
            app.velocity = uieditfield(app.InputPanel, 'numeric');
            app.velocity.HorizontalAlignment = 'center';
            app.velocity.Position = [116 129 68 22];

            % Create AngleofLaunchLabel
            app.AngleofLaunchLabel = uilabel(app.InputPanel);
            app.AngleofLaunchLabel.Position = [12 62 95 22];
            app.AngleofLaunchLabel.Text = 'Angle of Launch:';

            % Create angle
            app.angle = uieditfield(app.InputPanel, 'numeric');
            app.angle.HorizontalAlignment = 'center';
            app.angle.Position = [116 62 67 22];

            % Create IntialHeightLabel
            app.IntialHeightLabel = uilabel(app.InputPanel);
            app.IntialHeightLabel.Position = [12 96 75 22];
            app.IntialHeightLabel.Text = 'Initial Height:';

            % Create Hieght
            app.Hieght = uieditfield(app.InputPanel, 'numeric');
            app.Hieght.HorizontalAlignment = 'center';
            app.Hieght.Position = [116 96 67 22];

            % Create simulate
            app.simulate = uibutton(app.InputPanel, 'push');
            app.simulate.ButtonPushedFcn = createCallbackFcn(app, @simulateButtonPushed, true);
            app.simulate.Position = [344 99 100 41];
            app.simulate.Text = 'Simulate';

            % Create resetall
            app.resetall = uibutton(app.InputPanel, 'push');
            app.resetall.ButtonPushedFcn = createCallbackFcn(app, @resetallButtonPushed, true);
            app.resetall.Position = [344 30 100 40];
            app.resetall.Text = 'Reset All';

            % Create RTSSSwitchLabel
            app.RTSSSwitchLabel = uilabel(app.InputPanel);
            app.RTSSSwitchLabel.HorizontalAlignment = 'center';
            app.RTSSSwitchLabel.Tooltip = {'Real Time Simulator System'};
            app.RTSSSwitchLabel.Position = [274 42 35 22];
            app.RTSSSwitchLabel.Text = 'RTSS';

            % Create RTSSSwitch
            app.RTSSSwitch = uiswitch(app.InputPanel, 'slider');
            app.RTSSSwitch.Position = [270 79 41 18];

            % Create msLabel
            app.msLabel = uilabel(app.InputPanel);
            app.msLabel.HorizontalAlignment = 'center';
            app.msLabel.FontWeight = 'bold';
            app.msLabel.Position = [189 129 27 22];
            app.msLabel.Text = 'm/s';

            % Create mLabel
            app.mLabel = uilabel(app.InputPanel);
            app.mLabel.HorizontalAlignment = 'center';
            app.mLabel.FontWeight = 'bold';
            app.mLabel.Position = [191 96 25 22];
            app.mLabel.Text = 'm';

            % Create degLabel
            app.degLabel = uilabel(app.InputPanel);
            app.degLabel.HorizontalAlignment = 'center';
            app.degLabel.FontWeight = 'bold';
            app.degLabel.Position = [190 61 27 22];
            app.degLabel.Text = 'deg';

            % Create AccelerationLabel
            app.AccelerationLabel = uilabel(app.InputPanel);
            app.AccelerationLabel.Position = [12 30 75 22];
            app.AccelerationLabel.Text = 'Acceleration:';

            % Create acceleration
            app.acceleration = uidropdown(app.InputPanel);
            app.acceleration.Items = {'Earth', 'Moon', 'Mars'};
            app.acceleration.Position = [111 30 78 22];
            app.acceleration.Value = 'Earth';

            % Create OutputPanel
            app.OutputPanel = uipanel(app.simlayout);
            app.OutputPanel.Title = 'Output';
            app.OutputPanel.Layout.Row = [4 6];
            app.OutputPanel.Layout.Column = [1 2];
            app.OutputPanel.FontWeight = 'bold';

            % Create TimeofflightLabel
            app.TimeofflightLabel = uilabel(app.OutputPanel);
            app.TimeofflightLabel.Position = [12 127 78 22];
            app.TimeofflightLabel.Text = 'Time of flight:';

            % Create timeofflight
            app.timeofflight = uieditfield(app.OutputPanel, 'numeric');
            app.timeofflight.Editable = 'off';
            app.timeofflight.HorizontalAlignment = 'center';
            app.timeofflight.Position = [96 127 100 22];

            % Create RangeLabel
            app.RangeLabel = uilabel(app.OutputPanel);
            app.RangeLabel.Position = [14 32 43 22];
            app.RangeLabel.Text = 'Range:';

            % Create range
            app.range = uieditfield(app.OutputPanel, 'numeric');
            app.range.Editable = 'off';
            app.range.HorizontalAlignment = 'center';
            app.range.Position = [96 32 100 22];

            % Create PeakLocationEditFieldLabel
            app.PeakLocationEditFieldLabel = uilabel(app.OutputPanel);
            app.PeakLocationEditFieldLabel.Position = [12 96 85 22];
            app.PeakLocationEditFieldLabel.Text = 'Peak Location:';

            % Create peak
            app.peak = uieditfield(app.OutputPanel, 'text');
            app.peak.Editable = 'off';
            app.peak.HorizontalAlignment = 'center';
            app.peak.Placeholder = '(x,y)';
            app.peak.Position = [97 96 100 22];

            % Create MaxHieghtLabel
            app.MaxHieghtLabel = uilabel(app.OutputPanel);
            app.MaxHieghtLabel.Position = [12 64 70 22];
            app.MaxHieghtLabel.Text = 'Max Hieght:';

            % Create MH
            app.MH = uieditfield(app.OutputPanel, 'numeric');
            app.MH.Editable = 'off';
            app.MH.HorizontalAlignment = 'center';
            app.MH.Position = [96 64 101 22];

            % Create secLabel_2
            app.secLabel_2 = uilabel(app.OutputPanel);
            app.secLabel_2.HorizontalAlignment = 'center';
            app.secLabel_2.FontWeight = 'bold';
            app.secLabel_2.Position = [206 127 25 22];
            app.secLabel_2.Text = 'sec';

            % Create mLabel_2
            app.mLabel_2 = uilabel(app.OutputPanel);
            app.mLabel_2.HorizontalAlignment = 'center';
            app.mLabel_2.FontWeight = 'bold';
            app.mLabel_2.Position = [206 64 25 22];
            app.mLabel_2.Text = 'm';

            % Create mLabel_3
            app.mLabel_3 = uilabel(app.OutputPanel);
            app.mLabel_3.HorizontalAlignment = 'center';
            app.mLabel_3.FontWeight = 'bold';
            app.mLabel_3.Position = [206 32 25 22];
            app.mLabel_3.Text = 'm';

            % Create mLabel_4
            app.mLabel_4 = uilabel(app.OutputPanel);
            app.mLabel_4.HorizontalAlignment = 'center';
            app.mLabel_4.FontWeight = 'bold';
            app.mLabel_4.Position = [206 96 25 22];
            app.mLabel_4.Text = 'm';

            % Create LocationTrackerPanel
            app.LocationTrackerPanel = uipanel(app.simlayout);
            app.LocationTrackerPanel.Title = 'Location Tracker';
            app.LocationTrackerPanel.Layout.Row = [7 8];
            app.LocationTrackerPanel.Layout.Column = [1 2];
            app.LocationTrackerPanel.FontWeight = 'bold';

            % Create EnterthetimetocheckthelocationoftheobjectEditFieldLabel
            app.EnterthetimetocheckthelocationoftheobjectEditFieldLabel = uilabel(app.LocationTrackerPanel);
            app.EnterthetimetocheckthelocationoftheobjectEditFieldLabel.Position = [12 67 276 22];
            app.EnterthetimetocheckthelocationoftheobjectEditFieldLabel.Text = 'Enter the time to check the location of the object: ';

            % Create timelocate
            app.timelocate = uieditfield(app.LocationTrackerPanel, 'numeric');
            app.timelocate.Editable = 'off';
            app.timelocate.HorizontalAlignment = 'center';
            app.timelocate.Position = [286 67 100 22];

            % Create locationdisp
            app.locationdisp = uitextarea(app.LocationTrackerPanel);
            app.locationdisp.Editable = 'off';
            app.locationdisp.HorizontalAlignment = 'center';
            app.locationdisp.FontWeight = 'bold';
            app.locationdisp.BackgroundColor = [0.9412 0.9412 0.9412];
            app.locationdisp.Placeholder = 'At time t s, the object is at position x, y';
            app.locationdisp.Position = [12 12 264 40];

            % Create secLabel
            app.secLabel = uilabel(app.LocationTrackerPanel);
            app.secLabel.HorizontalAlignment = 'center';
            app.secLabel.FontWeight = 'bold';
            app.secLabel.Position = [393 67 25 22];
            app.secLabel.Text = 'sec';

            % Create locate
            app.locate = uibutton(app.LocationTrackerPanel, 'push');
            app.locate.ButtonPushedFcn = createCallbackFcn(app, @locateButtonPushed, true);
            app.locate.Enable = 'off';
            app.locate.Position = [286 22 100 23];
            app.locate.Text = 'Locate';

            % Create GraphsTab
            app.GraphsTab = uitab(app.TabGroup);
            app.GraphsTab.Title = 'Graphs';

            % Create GridLayout
            app.GridLayout = uigridlayout(app.GraphsTab);
            app.GridLayout.ColumnWidth = {'1x', '1x', '1x', '1x', '1x', '1x'};
            app.GridLayout.RowHeight = {'1x', '1x', '1x', '1x', '1x', '1x', '1x', '1x', '1x'};

            % Create DisplacementTimeGraph
            app.DisplacementTimeGraph = uiaxes(app.GridLayout);
            title(app.DisplacementTimeGraph, 'Displacement - Time')
            xlabel(app.DisplacementTimeGraph, 'Time (s)')
            ylabel(app.DisplacementTimeGraph, 'Displacement (m)')
            zlabel(app.DisplacementTimeGraph, 'Z')
            app.DisplacementTimeGraph.MinorGridLineStyle = '-';
            app.DisplacementTimeGraph.XGrid = 'on';
            app.DisplacementTimeGraph.XMinorGrid = 'on';
            app.DisplacementTimeGraph.YGrid = 'on';
            app.DisplacementTimeGraph.YMinorGrid = 'on';
            app.DisplacementTimeGraph.Layout.Row = [1 6];
            app.DisplacementTimeGraph.Layout.Column = [5 6];

            % Create VelocityTimeGraph
            app.VelocityTimeGraph = uiaxes(app.GridLayout);
            title(app.VelocityTimeGraph, 'Velocity - Time')
            xlabel(app.VelocityTimeGraph, 'Time (s)')
            ylabel(app.VelocityTimeGraph, 'Velocity (m/s)')
            zlabel(app.VelocityTimeGraph, 'Z')
            app.VelocityTimeGraph.MinorGridLineStyle = '-';
            app.VelocityTimeGraph.XGrid = 'on';
            app.VelocityTimeGraph.XMinorGrid = 'on';
            app.VelocityTimeGraph.YGrid = 'on';
            app.VelocityTimeGraph.YMinorGrid = 'on';
            app.VelocityTimeGraph.Layout.Row = [1 6];
            app.VelocityTimeGraph.Layout.Column = [3 4];

            % Create AccelerationTimeGraph
            app.AccelerationTimeGraph = uiaxes(app.GridLayout);
            title(app.AccelerationTimeGraph, 'Acceleration - Time')
            xlabel(app.AccelerationTimeGraph, 'Time (s)')
            ylabel(app.AccelerationTimeGraph, 'Acceleration (m/s²)')
            zlabel(app.AccelerationTimeGraph, 'Z')
            app.AccelerationTimeGraph.Toolbar.Visible = 'off';
            app.AccelerationTimeGraph.MinorGridLineStyle = '-';
            app.AccelerationTimeGraph.XGrid = 'on';
            app.AccelerationTimeGraph.XMinorGrid = 'on';
            app.AccelerationTimeGraph.YGrid = 'on';
            app.AccelerationTimeGraph.YMinorGrid = 'on';
            app.AccelerationTimeGraph.Layout.Row = [1 6];
            app.AccelerationTimeGraph.Layout.Column = [1 2];

            % Create MaxDisplacementLabel
            app.MaxDisplacementLabel = uilabel(app.GridLayout);
            app.MaxDisplacementLabel.HorizontalAlignment = 'center';
            app.MaxDisplacementLabel.Layout.Row = 7;
            app.MaxDisplacementLabel.Layout.Column = 5;
            app.MaxDisplacementLabel.Text = 'Maximum Displacement';

            % Create MaxDisplacement
            app.MaxDisplacement = uieditfield(app.GridLayout, 'numeric');
            app.MaxDisplacement.Editable = 'off';
            app.MaxDisplacement.HorizontalAlignment = 'center';
            app.MaxDisplacement.Layout.Row = 7;
            app.MaxDisplacement.Layout.Column = 6;

            % Create MaxDisplacementLabel_2
            app.MaxDisplacementLabel_2 = uilabel(app.GridLayout);
            app.MaxDisplacementLabel_2.HorizontalAlignment = 'center';
            app.MaxDisplacementLabel_2.Layout.Row = 8;
            app.MaxDisplacementLabel_2.Layout.Column = 5;
            app.MaxDisplacementLabel_2.Text = 'Minimum Displacement';

            % Create MinDisplacement
            app.MinDisplacement = uieditfield(app.GridLayout, 'numeric');
            app.MinDisplacement.Editable = 'off';
            app.MinDisplacement.HorizontalAlignment = 'center';
            app.MinDisplacement.Layout.Row = 8;
            app.MinDisplacement.Layout.Column = 6;

            % Create MaxDisplacementLabel_3
            app.MaxDisplacementLabel_3 = uilabel(app.GridLayout);
            app.MaxDisplacementLabel_3.HorizontalAlignment = 'center';
            app.MaxDisplacementLabel_3.Layout.Row = 9;
            app.MaxDisplacementLabel_3.Layout.Column = 5;
            app.MaxDisplacementLabel_3.Text = 'Average Velocity (m/s)';

            % Create AvgVelocity
            app.AvgVelocity = uieditfield(app.GridLayout, 'numeric');
            app.AvgVelocity.Editable = 'off';
            app.AvgVelocity.HorizontalAlignment = 'center';
            app.AvgVelocity.Layout.Row = 9;
            app.AvgVelocity.Layout.Column = 6;

            % Create MaxDisplacementLabel_4
            app.MaxDisplacementLabel_4 = uilabel(app.GridLayout);
            app.MaxDisplacementLabel_4.HorizontalAlignment = 'center';
            app.MaxDisplacementLabel_4.Layout.Row = 7;
            app.MaxDisplacementLabel_4.Layout.Column = 3;
            app.MaxDisplacementLabel_4.Text = 'Maximum Velocity';

            % Create MaxVelocity
            app.MaxVelocity = uieditfield(app.GridLayout, 'numeric');
            app.MaxVelocity.Editable = 'off';
            app.MaxVelocity.HorizontalAlignment = 'center';
            app.MaxVelocity.Layout.Row = 7;
            app.MaxVelocity.Layout.Column = 4;

            % Create MaxDisplacementLabel_5
            app.MaxDisplacementLabel_5 = uilabel(app.GridLayout);
            app.MaxDisplacementLabel_5.HorizontalAlignment = 'center';
            app.MaxDisplacementLabel_5.Layout.Row = 8;
            app.MaxDisplacementLabel_5.Layout.Column = 3;
            app.MaxDisplacementLabel_5.Text = 'Minimum Velocity';

            % Create MinVelocity
            app.MinVelocity = uieditfield(app.GridLayout, 'numeric');
            app.MinVelocity.Editable = 'off';
            app.MinVelocity.HorizontalAlignment = 'center';
            app.MinVelocity.Layout.Row = 8;
            app.MinVelocity.Layout.Column = 4;

            % Create MaxDisplacementLabel_7
            app.MaxDisplacementLabel_7 = uilabel(app.GridLayout);
            app.MaxDisplacementLabel_7.HorizontalAlignment = 'center';
            app.MaxDisplacementLabel_7.Layout.Row = 9;
            app.MaxDisplacementLabel_7.Layout.Column = 3;
            app.MaxDisplacementLabel_7.Text = 'Average Acceleration (m/s²)';

            % Create AvgAcceleration
            app.AvgAcceleration = uieditfield(app.GridLayout, 'numeric');
            app.AvgAcceleration.Editable = 'off';
            app.AvgAcceleration.HorizontalAlignment = 'center';
            app.AvgAcceleration.Layout.Row = 9;
            app.AvgAcceleration.Layout.Column = 4;

            % Create MaxDisplacementLabel_8
            app.MaxDisplacementLabel_8 = uilabel(app.GridLayout);
            app.MaxDisplacementLabel_8.HorizontalAlignment = 'center';
            app.MaxDisplacementLabel_8.Layout.Row = 7;
            app.MaxDisplacementLabel_8.Layout.Column = 1;
            app.MaxDisplacementLabel_8.Text = 'Maximum Acceleration';

            % Create MaxAcceleration
            app.MaxAcceleration = uieditfield(app.GridLayout, 'numeric');
            app.MaxAcceleration.Editable = 'off';
            app.MaxAcceleration.HorizontalAlignment = 'center';
            app.MaxAcceleration.Layout.Row = 7;
            app.MaxAcceleration.Layout.Column = 2;

            % Create MaxDisplacementLabel_9
            app.MaxDisplacementLabel_9 = uilabel(app.GridLayout);
            app.MaxDisplacementLabel_9.HorizontalAlignment = 'center';
            app.MaxDisplacementLabel_9.Layout.Row = 8;
            app.MaxDisplacementLabel_9.Layout.Column = 1;
            app.MaxDisplacementLabel_9.Text = 'Minimum Acceleration';

            % Create MinAcceleration
            app.MinAcceleration = uieditfield(app.GridLayout, 'numeric');
            app.MinAcceleration.Editable = 'off';
            app.MinAcceleration.HorizontalAlignment = 'center';
            app.MinAcceleration.Layout.Row = 8;
            app.MinAcceleration.Layout.Column = 2;

            % Create MaxDisplacementLabel_10
            app.MaxDisplacementLabel_10 = uilabel(app.GridLayout);
            app.MaxDisplacementLabel_10.HorizontalAlignment = 'center';
            app.MaxDisplacementLabel_10.Layout.Row = 9;
            app.MaxDisplacementLabel_10.Layout.Column = 1;
            app.MaxDisplacementLabel_10.Text = 'Jerk (m/s³)';

            % Create Jerk
            app.Jerk = uieditfield(app.GridLayout, 'numeric');
            app.Jerk.Editable = 'off';
            app.Jerk.HorizontalAlignment = 'center';
            app.Jerk.Layout.Row = 9;
            app.Jerk.Layout.Column = 2;

            % Create HistoryTab
            app.HistoryTab = uitab(app.TabGroup);
            app.HistoryTab.Title = 'History';

            % Create ProjectileSimulatorLabel
            app.ProjectileSimulatorLabel = uilabel(app.UIFigure);
            app.ProjectileSimulatorLabel.HorizontalAlignment = 'center';
            app.ProjectileSimulatorLabel.FontSize = 36;
            app.ProjectileSimulatorLabel.FontWeight = 'bold';
            app.ProjectileSimulatorLabel.Position = [305 553 348 48];
            app.ProjectileSimulatorLabel.Text = 'Projectile Simulator';

            % Create Versionlabel
            app.Versionlabel = uilabel(app.UIFigure);
            app.Versionlabel.HorizontalAlignment = 'center';
            app.Versionlabel.FontWeight = 'bold';
            app.Versionlabel.Position = [922 579 25 22];
            app.Versionlabel.Text = 'V2';

            % Show the figure after all components are created
            app.UIFigure.Visible = 'on';
        end
    end

    % App creation and deletion
    methods (Access = public)

        % Construct app
        function app = Projectile_Simulator_TP_exported

            % Create UIFigure and components
            createComponents(app)

            % Register the app with App Designer
            registerApp(app, app.UIFigure)

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