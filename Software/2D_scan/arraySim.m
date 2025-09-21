results_filename = 'antenna_patterns.mat';

SubObject = design(patchMicrostrip, 2.4e9);
arrayObject = design(rectangularArray('Element', SubObject), 2.4e9, SubObject);
arrayObject.Element.Load.Impedance = 50;
arrayObject.Size = [2, 4];
plotFrequency = 2.4e9;

% Define the range of steering angles.
az_angles = -80:5:80;
el_angles = 0:5:160;

% Start a parallel pool of workers (if not already started).
if isempty(gcp('nocreate'))
    parpool;
end

% Pre-calculate the grid to ensure consistent dimensions.
% We get the azimuth (az_grid) and elevation (el_grid) vectors.
[~, az_grid, el_grid] = pattern(arrayObject, plotFrequency);

% Pre-allocate a 3D matrix to store all pattern data.
% Dimensions: (elevation points x azimuth points x number of angles)
pattern_data_storage = zeros(numel(el_grid), numel(az_grid), numel(az_angles), numel(el_angles));

% --- Parallel Calculation Loop  ---
disp('Calculating all radiation patterns in parallel...');

parfor i = 1:numel(az_angles)
    % Create a temporary variable to hold the results for the inner loop.
    % This variable is local to each parallel worker.
    temp_el_slice = zeros(numel(el_grid), numel(az_grid), numel(el_angles));

    for j = 1:numel(el_angles)
        current_az = az_angles(i);
        current_el = el_angles(j);
    
        % Create a local copy of the arrayObject for each worker.
        localArray = arrayObject;
    
        % Calculate the phase shift.
        ps = phaseShift(localArray, plotFrequency, [current_az, current_el])';
        localArray.PhaseShift = ps;
            
        % Calculate the pattern without plotting.
        [directivity_data, ~, ~] = pattern(localArray, plotFrequency, 'Azimuth', az_grid, 'Elevation', el_grid);
        
        % Store the result in the temporary matrix, not the main one.
        temp_el_slice(:, :, j) = directivity_data;
        
        % This fprintf might be messy in parallel, but we'll leave it for now.
        % It's often better to check progress in other ways for parfor loops.
        fprintf('Calculating for [Az, El]: [%d°, %d°]\n', current_az, current_el);
    end
    
    % After the inner loop is done, assign the entire temporary matrix
    % to the correct "slice" of the main storage variable. This is a valid
    % operation in a parfor loop.
    pattern_data_storage(:, :, i, :) = temp_el_slice;
end

disp('All pattern calculations complete.');
fprintf('Saving to file: %s\n', results_filename);

save(results_filename, 'pattern_data_storage', 'az_angles', 'el_angles', 'az_grid', 'el_grid');
disp('Data saved.');