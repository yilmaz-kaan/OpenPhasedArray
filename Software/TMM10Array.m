%% 1. Define Design Parameters
% Define the operating frequency (e.g., 2.5 GHz).
% All calculations (patch size, spacing) depend on this.
f_op = 3.1e9; % 2.5 GHz
c = physconst('lightspeed');
lambda = c / f_op;

% Calculate the requested quarter-wave spacing
quarter_wave_spacing = lambda / 4;

fprintf('Operating Frequency: %.2f GHz\n', f_op/1e9);
fprintf('Quarter-Wave Spacing: %.2f mm\n', quarter_wave_spacing * 1000);

%% 2. Define TMM10 Dielectric
% As requested, we use TMM10.
% A common thickness for TMM10 is 1.27mm (50 mils).
d_tmm10 = dielectric('TMM10');
d_tmm10.Thickness = 1.27e-3; 

%% 3. Design a Single Patch Element for TMM10
% We must design a patch that resonates at f_op on the TMM10 substrate.
% We use the 'design' function to get the correct dimensions.
patch_element = design(patchMicrostrip('Substrate', d_tmm10), f_op);

% Define the ground plane size for the single element.
% We'll set it to be exactly the size of the spacing, so the
% ground planes of adjacent elements will touch.
patch_element.GroundPlaneLength = quarter_wave_spacing;
patch_element.GroundPlaneWidth = quarter_wave_spacing;

%% 4. Create the [2 4] Antenna Array
% We follow the guide's method using 'rectangularArray' but
% substitute your specified parameters.
array_2x4 = rectangularArray;
array_2x4.Element = patch_element;
array_2x4.Size = [2 4]; % User-specified [2 4] size
array_2x4.RowSpacing = quarter_wave_spacing; % User-specified spacing
array_2x4.ColumnSpacing = quarter_wave_spacing; % User-specified spacing

%% 5. Visualize the Array
figure;
show(array_2x4);
title('2x4 Patch Array with Quarter-Wave Spacing on TMM10');

% You can also create the full pcbStack, as shown in the guide,
% to prepare for analysis or manufacturing.
ant_pcb = pcbStack(array_2x4);
figure;
show(ant_pcb);
title('PCB Stack View of 2x4 Array');


freq_range = linspace(2.0e9, 3.0e9, 101);
% Define the reference impedance (typically 50 Ohms)
z0 = 50;

% --- 6a. Impedance
% This calculates and plots the impedance for all 8 ports.
figure;
impedance(ant_pcb, freq_range);
title('Input Impedance (All 8 Ports)');

% --- 6b. S-Parameters and Return Loss
% Calculate the S-parameters. This will be an 8x8 matrix
% (S11, S21, S31... S88) at each frequency.
spar = sparameters(ant_pcb, freq_range, z0);

% Plot the return loss (S11, S22, ... S88) for all ports.
figure;
returnLoss(ant_pcb, freq_range, z0);
title('Return Loss (All 8 Ports)');

% To plot S-parameters using rfplot:
% figure;
% rfplot(spar, 1, 1); % Plots S11
% hold on;
% rfplot(spar, 2, 2); % Plots S22
% ...and so on for all 8 ports
% title('S11 and S22');
% legend('S11', 'S22');

% --- 6c. 3D Radiation Pattern
% This calculates and plots the full 3D directivity pattern
% at the operating frequency.
figure;
pattern(ant_pcb, f_op);
title('3D Radiation Pattern');

% --- 6d. Azimuth and Elevation Patterns (2D Slices)
% Plot the Azimuth (horizontal) pattern
% This is a slice at 0 degrees elevation (the "horizon")
figure;
pattern(ant_pcb, f_op, 'Azimuth', -180:1:180, 'Elevation', 0);
title('Azimuth Pattern (Elevation = 0°)');

% Plot the Elevation (vertical) pattern
% This is a slice at 0 degrees azimuth (the "front")
figure;
pattern(ant_pcb, f_op, 'Azimuth', 0, 'Elevation', -90:1:90);
title('Elevation Pattern (Azimuth = 0°)');