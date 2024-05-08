classdef ENGG100_MVP_Code < matlab.apps.AppBase

    % Properties that correspond to app components
    properties (Access = public)
        UIFigure                  matlab.ui.Figure
        versionlabel              matlab.ui.control.Label
        maxhlablel                matlab.ui.control.Label
        anglelabel                matlab.ui.control.Label
        velocitylabel             matlab.ui.control.Label
        timelabel                 matlab.ui.control.Label
        hlabel                    matlab.ui.control.Label
        dlabel                    matlab.ui.control.Label
        MainTitle                 matlab.ui.control.Label
        locationdisp              matlab.ui.control.TextArea
        locate                    matlab.ui.control.Button
        timelocate                matlab.ui.control.NumericEditField
        EnterthetimetocheckthelocationoftheobjectEditFieldLabel  matlab.ui.control.Label
        MH                        matlab.ui.control.NumericEditField
        MaxHieghtmEditFieldLabel  matlab.ui.control.Label
        angle                     matlab.ui.control.NumericEditField
        AnglealphaEditFieldLabel  matlab.ui.control.Label
        cal                       matlab.ui.control.Button
        velocity                  matlab.ui.control.NumericEditField
        VelocityVLabel            matlab.ui.control.Label
        hieght                    matlab.ui.control.NumericEditField
        EntertheheightofthebuildingHLabel  matlab.ui.control.Label
        distance                  matlab.ui.control.NumericEditField
        EnterthedistancefromthebuildingDEditFieldLabel  matlab.ui.control.Label
        graph                     matlab.ui.control.UIAxes
    end

    
    properties (Access = private)
               % Constants
               g = 24.79; % Acceleration due to gravity (m/s^2)
               basketball_ring_height = 3; % Height of the basketball ring (m)
               basketball_ring_distance = 6; % Distance of the basketball ring from the building (m)
    end
    
    methods (Access = private)

        % Custom Functions

        function [V,alpha] = Velangle(app) % The function calculates the Velocity and the angle of the projectile while displaying them
          
            D = app.distance.Value; % Distance field
            H = app.hieght.Value; % Hieght field
            alpha = atan((H - app.basketball_ring_height) / D); % Angle
            V = sqrt((D^2 * app.g) / (D * tan(asin((H - app.basketball_ring_height)/D)) - H + app.basketball_ring_height)); % Velocity
            app.velocity.Value = double(V) ; % Displays Velocity
            app.angle.Value = alpha; % Displays Angle

            disp(V); % CLI PRINT V FOR DEBUG

          
           
        end
        
        function maxhieght = maxh(app) % The function calculates the max hieght of the projectile and displays them
        
            [V,alpha] = Velangle(app); 
            t_max_height = V * sin(alpha) / app.g; % t max height 
            y_max_height = V * sin(alpha) * t_max_height - 0.5 * app.g * t_max_height^2; % Max Hieght
            app.MH.Value = y_max_height; % Displays Max Hieght

         
            

        end
        
        function gr = plotgraph(app) % The function Plots the graph in the graph field
           
                [V,alpha] = Velangle(app); 
                t = linspace(0, 2 * V * sin(alpha) / app.g, 100);
                x = V * cos(alpha) * t; % X - Axis 
                y = V * sin(alpha) * t - 0.5 * app.g * t.^2; % Y - Axis
                plot(app.graph,x,y) % plots the graph
            

          

        end
        
        function lw = locatew(app) % The function locates the projectile in x,y coordinaties in a given time period  
      
            [V,alpha] = Velangle(app);
            t_check = app.timelocate.Value; % Time field
            x_check = V * cos(alpha) * t_check; % X - Location
            y_check = V * sin(alpha) * t_check - 0.5 * app.g * t_check^2; % Y - Location
            app.locationdisp.Value= ['At time ' num2str(t_check) ' s, the object is at position (' num2str(x_check) ', ' num2str(y_check) ')'];  % Displays time, (x,y) coordinates in locationdisp
        
    
        end
    end
    

    % Callbacks that handle component events
    methods (Access = private)

        % Button pushed function: cal
        function Calcl(app, event)
           try
            Velangle(app); % Calls Calculate function
            plotgraph(app); % Calls the Plot Graph function
            maxh(app); % Calls the max hieght function
           catch ME
               error = ME.identifier; % Shows the MATLAB Identifier and assigns it to variable error
               helpdlg([error,"These Values are not possible, Please try diffrent values"],"ERROR"); % Opens an Help Alert Dialog and shows the error to the user
           end
            
            
        end

        % Button pushed function: locate
        function locateB(app, event)
            locatew(app); % Calls the locate projectile function
        end
    end

    % Component initialization
    methods (Access = private)

        % Create UIFigure and components
        function createComponents(app)

            % Create UIFigure and hide until all components are created
            app.UIFigure = uifigure('Visible', 'off');
            app.UIFigure.Position = [100 100 972 584];
            app.UIFigure.Name = 'MATLAB App';

            % Create graph
            app.graph = uiaxes(app.UIFigure);
            title(app.graph, 'Projectile Motion Graph')
            xlabel(app.graph, 'Horizontal Distance (m)')
            ylabel(app.graph, 'Vertical Distance (m)')
            zlabel(app.graph, 'Z')
            app.graph.Position = [560 115 399 398];

            % Create EnterthedistancefromthebuildingDEditFieldLabel
            app.EnterthedistancefromthebuildingDEditFieldLabel = uilabel(app.UIFigure);
            app.EnterthedistancefromthebuildingDEditFieldLabel.HorizontalAlignment = 'right';
            app.EnterthedistancefromthebuildingDEditFieldLabel.Position = [30 491 219 22];
            app.EnterthedistancefromthebuildingDEditFieldLabel.Text = 'Enter the distance from the building (D):';

            % Create distance
            app.distance = uieditfield(app.UIFigure, 'numeric');
            app.distance.HorizontalAlignment = 'center';
            app.distance.Position = [264 484 191 36];

            % Create EntertheheightofthebuildingHLabel
            app.EntertheheightofthebuildingHLabel = uilabel(app.UIFigure);
            app.EntertheheightofthebuildingHLabel.HorizontalAlignment = 'right';
            app.EntertheheightofthebuildingHLabel.Position = [31 442 192 22];
            app.EntertheheightofthebuildingHLabel.Text = 'Enter the height of the building (H):';

            % Create hieght
            app.hieght = uieditfield(app.UIFigure, 'numeric');
            app.hieght.HorizontalAlignment = 'center';
            app.hieght.Position = [238 441 214 23];

            % Create VelocityVLabel
            app.VelocityVLabel = uilabel(app.UIFigure);
            app.VelocityVLabel.HorizontalAlignment = 'center';
            app.VelocityVLabel.Position = [34 228 63 22];
            app.VelocityVLabel.Text = 'Velocity (V)';

            % Create velocity
            app.velocity = uieditfield(app.UIFigure, 'numeric');
            app.velocity.Editable = 'off';
            app.velocity.HorizontalAlignment = 'center';
            app.velocity.Position = [137 213 342 52];

            % Create cal
            app.cal = uibutton(app.UIFigure, 'push');
            app.cal.ButtonPushedFcn = createCallbackFcn(app, @Calcl, true);
            app.cal.Position = [50 301 133 59];
            app.cal.Text = 'Calculate';

            % Create AnglealphaEditFieldLabel
            app.AnglealphaEditFieldLabel = uilabel(app.UIFigure);
            app.AnglealphaEditFieldLabel.HorizontalAlignment = 'right';
            app.AnglealphaEditFieldLabel.Position = [33 156 78 22];
            app.AnglealphaEditFieldLabel.Text = 'Angle (alpha):';

            % Create angle
            app.angle = uieditfield(app.UIFigure, 'numeric');
            app.angle.Editable = 'off';
            app.angle.HorizontalAlignment = 'center';
            app.angle.Placeholder = 'α';
            app.angle.Position = [138 139 341 55];

            % Create MaxHieghtmEditFieldLabel
            app.MaxHieghtmEditFieldLabel = uilabel(app.UIFigure);
            app.MaxHieghtmEditFieldLabel.BackgroundColor = [0.9412 0.9412 0.9412];
            app.MaxHieghtmEditFieldLabel.HorizontalAlignment = 'right';
            app.MaxHieghtmEditFieldLabel.Position = [37 88 86 22];
            app.MaxHieghtmEditFieldLabel.Text = 'Max Hieght (m)';

            % Create MH
            app.MH = uieditfield(app.UIFigure, 'numeric');
            app.MH.Editable = 'off';
            app.MH.HorizontalAlignment = 'center';
            app.MH.BackgroundColor = [0.9412 0.9412 0.9412];
            app.MH.Position = [139 71 340 55];

            % Create EnterthetimetocheckthelocationoftheobjectEditFieldLabel
            app.EnterthetimetocheckthelocationoftheobjectEditFieldLabel = uilabel(app.UIFigure);
            app.EnterthetimetocheckthelocationoftheobjectEditFieldLabel.HorizontalAlignment = 'right';
            app.EnterthetimetocheckthelocationoftheobjectEditFieldLabel.Position = [27 397 276 22];
            app.EnterthetimetocheckthelocationoftheobjectEditFieldLabel.Text = 'Enter the time to check the location of the object: ';

            % Create timelocate
            app.timelocate = uieditfield(app.UIFigure, 'numeric');
            app.timelocate.HorizontalAlignment = 'center';
            app.timelocate.Position = [341 385 115 46];

            % Create locate
            app.locate = uibutton(app.UIFigure, 'push');
            app.locate.ButtonPushedFcn = createCallbackFcn(app, @locateB, true);
            app.locate.Position = [222 301 133 59];
            app.locate.Text = 'Locate';

            % Create locationdisp
            app.locationdisp = uitextarea(app.UIFigure);
            app.locationdisp.Editable = 'off';
            app.locationdisp.HorizontalAlignment = 'center';
            app.locationdisp.FontWeight = 'bold';
            app.locationdisp.BackgroundColor = [0.9412 0.9412 0.9412];
            app.locationdisp.Placeholder = 'At time t s, the object is at position x, y';
            app.locationdisp.Position = [37 24 558 31];

            % Create MainTitle
            app.MainTitle = uilabel(app.UIFigure);
            app.MainTitle.HorizontalAlignment = 'center';
            app.MainTitle.WordWrap = 'on';
            app.MainTitle.FontName = 'Arial';
            app.MainTitle.FontSize = 24;
            app.MainTitle.FontWeight = 'bold';
            app.MainTitle.Position = [196 537 586 32];
            app.MainTitle.Text = 'ENGG100 Project MVP';

            % Create dlabel
            app.dlabel = uilabel(app.UIFigure);
            app.dlabel.HorizontalAlignment = 'center';
            app.dlabel.Position = [460 491 25 22];
            app.dlabel.Text = 'm';

            % Create hlabel
            app.hlabel = uilabel(app.UIFigure);
            app.hlabel.HorizontalAlignment = 'center';
            app.hlabel.Position = [460 442 25 22];
            app.hlabel.Text = 'm';

            % Create timelabel
            app.timelabel = uilabel(app.UIFigure);
            app.timelabel.HorizontalAlignment = 'center';
            app.timelabel.Position = [460 387 25 22];
            app.timelabel.Text = 'sec';

            % Create velocitylabel
            app.velocitylabel = uilabel(app.UIFigure);
            app.velocitylabel.HorizontalAlignment = 'center';
            app.velocitylabel.Position = [485 227 25 22];
            app.velocitylabel.Text = 'm/s';

            % Create anglelabel
            app.anglelabel = uilabel(app.UIFigure);
            app.anglelabel.HorizontalAlignment = 'center';
            app.anglelabel.Position = [485 155 48 22];
            app.anglelabel.Text = 'degrees';

            % Create maxhlablel
            app.maxhlablel = uilabel(app.UIFigure);
            app.maxhlablel.HorizontalAlignment = 'center';
            app.maxhlablel.Position = [485 87 25 22];
            app.maxhlablel.Text = 'm';

            % Create versionlabel
            app.versionlabel = uilabel(app.UIFigure);
            app.versionlabel.HorizontalAlignment = 'center';
            app.versionlabel.FontWeight = 'bold';
            app.versionlabel.Position = [923 3 62 22];
            app.versionlabel.Text = 'v1.0';

            % Show the figure after all components are created
            app.UIFigure.Visible = 'on';
        end
    end

    % App creation and deletion
    methods (Access = public)

        % Construct app
        function app = ENGG100_MVP_Code

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