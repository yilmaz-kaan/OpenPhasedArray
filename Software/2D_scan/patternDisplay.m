%==========================================================================
% SCRIPT: patternDisplay.m
% PURPOSE: Interactively steer a 3D antenna pattern by dragging a marker
%          on a 2D Cartesian (X-Y) controller plot.
%==========================================================================
function patternDisplay
    clear; clc; close all;

    results_filename = 'antenna_patterns.mat';

    %% Load or Generate Simulation Data
    if exist(results_filename, 'file')
        fprintf('Loading pre-computed data from %s\n', results_filename);
        s = load(results_filename);
        disp('Data loaded successfully.');
    else
        disp('Data file not found. Running simulation...');
        arraySim;
        s = load(results_filename);
    end

    %% Pre-computation & GUI State
    s.min_directivity = min(s.pattern_data_storage, [], 'all');
    [s.AZ_grid, s.EL_grid] = meshgrid(s.az_grid, s.el_grid);
    
    state.isDragging = false;
    state.az_index = 1;
    state.el_index = 1;
    
    %% Create the GUI Layout
    fig = figure('Name', 'Interactive XY Beamsteering', 'Position', [100, 100, 1200, 600]);
    ax_3d = axes('Parent', fig, 'Position', [0.08, 0.1, 0.4, 0.8]);
    ax_controller = axes('Parent', fig, 'Position', [0.58, 0.15, 0.38, 0.75]);
    
    %% Draw the Initial Plots
    axis_limit = calculateAxisLimit(s);

    initial_directivity = s.pattern_data_storage(:,:,1,1);
    radius_initial = initial_directivity - s.min_directivity;
    [X_initial, Y_initial, Z_initial] = sph2cart(deg2rad(s.AZ_grid), deg2rad(s.EL_grid), radius_initial);
    h_surf = surf(ax_3d, X_initial, Y_initial, Z_initial, initial_directivity);
    
    shading(ax_3d, 'interp');
    axis(ax_3d, 'equal');
    xlabel(ax_3d, 'X'); ylabel(ax_3d, 'Y'); zlabel(ax_3d, 'Z');
    grid(ax_3d, 'on');
    axis(ax_3d, [-axis_limit, axis_limit, -axis_limit, axis_limit, -axis_limit, axis_limit]);
    c = colorbar(ax_3d);
    c.Label.String = 'Directivity (dBi)';
    clim(ax_3d, [min(s.pattern_data_storage,[],'all'), max(s.pattern_data_storage,[],'all')]);
    view(ax_3d, -35, 30);

    hold(ax_controller, 'on');
    [az_grid_points, el_grid_points] = meshgrid(s.az_angles, s.el_angles);
    plot(ax_controller, az_grid_points, el_grid_points, '.', 'Color', [0.5 0.5 0.5], 'HandleVisibility', 'off', 'PickableParts', 'none');
    h_target = plot(ax_controller, 0, 0, 'bo', 'MarkerSize', 10, 'LineWidth', 2, 'MarkerFaceColor', 'b');
    hold(ax_controller, 'off');
    
    grid(ax_controller, 'on');
    xlabel(ax_controller, 'Azimuth Steering (°)');
    ylabel(ax_controller, 'Elevation Steering (°)');
    title(ax_controller, 'Click and Drag to Steer Beam');
    xlim(ax_controller, [min(s.az_angles)-5, max(s.az_angles)+5]);
    ylim(ax_controller, [min(s.el_angles)-5, max(s.el_angles)+5]);
    axis(ax_controller, 'equal');

    handles.surf = h_surf;
    handles.target = h_target;
    handles.ax_3d = ax_3d;
    handles.ax_controller = ax_controller;

    %% Set up Mouse Interaction Callbacks
    % This makes it the primary listener for clicks within its bounds.
    set(handles.ax_controller, 'ButtonDownFcn', @onMouseDown);
    
    % The motion and up functions remain on the main figure so that dragging
    % continues to work even if the mouse leaves the controller's area.
    set(fig, 'WindowButtonMotionFcn', @onMouseMove);
    set(fig, 'WindowButtonUpFcn', @onMouseUp);

    updatePlots(1, 1);

    %% Nested Callback Functions
    function onMouseDown(~, ~)
        % This function now ONLY runs when the click is on the controller.
        % Its only job is to start the drag operation.
        state.isDragging = true;
        onMouseMove(); % Trigger an update immediately on click
    end
    
    function onMouseUp(~, ~)
        state.isDragging = false;
    end

    function onMouseMove(~, ~)
        if ~state.isDragging, return; end
        
        mousePoint = get(handles.ax_controller, 'CurrentPoint');
        desired_az = mousePoint(1,1);
        desired_el = mousePoint(1,2);
        
        [~, new_az_index] = min(abs(s.az_angles - desired_az));
        [~, new_el_index] = min(abs(s.el_angles - desired_el));
        
        if new_az_index ~= state.az_index || new_el_index ~= state.el_index
            state.az_index = new_az_index;
            state.el_index = new_el_index;
            updatePlots(state.az_index, state.el_index);
        end
    end

    function updatePlots(az_idx, el_idx)
        current_az = s.az_angles(az_idx);
        current_el = s.el_angles(el_idx);
        current_directivity = s.pattern_data_storage(:, :, az_idx, el_idx);
        
        radius = current_directivity - s.min_directivity;
        [X, Y, Z] = sph2cart(deg2rad(s.AZ_grid), deg2rad(s.EL_grid), radius);
        set(handles.surf, 'XData', X, 'YData', Y, 'ZData', Z, 'CData', current_directivity);
        title(handles.ax_3d, ['Pattern Steered to [Az, El]: [', num2str(current_az), '°, ', num2str(current_el), '°]']);
        
        set(handles.target, 'XData', current_az, 'YData', current_el);
        drawnow;
    end

    function limit = calculateAxisLimit(data_struct)
        limit = 0;
        for i = 1:numel(data_struct.az_angles)
            for j = 1:numel(data_struct.el_angles)
                rad = data_struct.pattern_data_storage(:,:,i,j) - data_struct.min_directivity;
                [X, Y, Z] = sph2cart(deg2rad(data_struct.AZ_grid), deg2rad(data_struct.EL_grid), rad);
                current_max = max(abs([X(:); Y(:); Z(:)]));
                if current_max > limit
                    limit = current_max;
                end
            end
        end
        limit = limit * 1.1;
    end
end