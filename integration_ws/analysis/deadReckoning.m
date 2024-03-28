% run this using: sudo matlab -softwareopengl
% Select a ROS bag file and topic
shape = 'circle';
addpath('~/Documents/MATLAB');
plot_path = '../data/plots/town/'

% Use sprintf to dynamically create the file path
filename = sprintf('../data/%s/%s_imu.bag', shape, shape);
bag_select = rosbag(filename); % load the bag file
bSel = select(bag_select, 'Topic', '/imu'); % select the topic associated with IMU data

set(0, 'DefaultFigureVisible', 'off')

% Read messages from the selected bag file and topic
msg_struct = readMessages(bSel, 'DataFormat', 'struct');
disp(fieldnames(msg_struct{1}.MagField))

% Plot the raw and corrected yaw angles for comparison
figWidthInches = 12; 
figHeightInches = 4;

% Convert size to pixels (assuming 96 DPI)
figWidthPixels = figWidthInches * 96;
figHeightPixels = figHeightInches * 96;

% Step 0. Get parameters of calibration (soft and hard iron correction)
% Extracting time circles
sec = cellfun(@(m) double(m.Header.Stamp.Sec), msg_struct); % extract seconds from the header stamp
nsec = cellfun(@(m) double(m.Header.Stamp.Nsec), msg_struct); % extract nanoseconds from the header stamp
time_circles = sec - min(sec) + nsec*1e-9; % calculate time in circles with nanosecond precision

% Extract magnetic field data from the messages
magcircles_x = cellfun(@(m) double(m.MagField.MagneticField_.X), msg_struct); % extract X component
magcircles_y = cellfun(@(m) double(m.MagField.MagneticField_.Y), msg_struct); % extract Y component
magcircles_z = cellfun(@(m) double(m.MagField.MagneticField_.Z), msg_struct); % extract Z component

% Fit ellipse to the magnetic field data
ell = fit_ellipse(magcircles_x, magcircles_y); % fit an ellipse to the X and Y components

% Apply hard and soft corrections on town in circles data
phi = ell.phi; % angle of the ellipse rotation
scale_mat = [1/ell.long_axis, 0; 0, 1/ell.short_axis]; % scale matrix for soft-iron correction
rotation = [cos(phi), -sin(phi); sin(phi), cos(phi)]; % rotation matrix for hard-iron correction
corr_x = magcircles_x - ell.X0_in; % apply hard-iron correction on X
corr_y = magcircles_y - ell.Y0_in; % apply hard-iron correction on Y
corr_coords = [corr_x, corr_y]; % corrected coordinates matrix
corr_coords = transpose(corr_coords); % transpose for matrix multiplication
circle_ellipse = transpose(scale_mat*rotation*corr_coords); % apply soft-iron correction

% Fit a new ellipse to the corrected data
new_ell = fit_ellipse(circle_ellipse(:,1), circle_ellipse(:,2)); % fit an ellipse to the corrected data

% Plot 0: Magnet Field before and after correction.
% Compute heading (yaw angle) using calibrated magnetometer x and y.
% the yaw angle typically represents the heading of a vehicle or object.
% Load the town data bag file
bag_filename_town = '../data/town/town_imu.bag';
bag_select_town = rosbag(bag_filename_town); % load the bag file
bSel_town = select(bag_select_town, 'Topic', '/imu'); % select the topic associated with IMU data

% Read messages from the selected bag file and topic
msg_struct_town = readMessages(bSel_town, 'DataFormat', 'struct');

% Extract magnetic field data from the messages
magtown_x = cellfun(@(m) double(m.MagField.MagneticField_.X), msg_struct_town);
magtown_y = cellfun(@(m) double(m.MagField.MagneticField_.Y), msg_struct_town);

% Correct the magnetic field data using the previously calculated parameters
corr_x_town = magtown_x - ell.X0_in;
corr_y_town = magtown_y - ell.Y0_in;
corr_coords_town = [corr_x_town, corr_y_town];
corr_coords_town = transpose(corr_coords_town);
corrected_town = transpose(scale_mat*rotation*corr_coords_town);

figure;
scatter(magtown_x, magtown_y, 'r*'); hold on;  % Original data in red
scatter(corrected_town(:,1), corrected_town(:,2), 'b*');  % Corrected data in blue
legend('Before Correction', 'After Correction');
title('Magnetic Field Data: Before and After Correction');
xlabel('Magnetic Field X (Gauss)');
ylabel('Magnetic Field Y (Gauss)');
grid on;
filename = 'plot_0_magnet_before_after_correction_town.png';
full_path = fullfile(plot_path, filename);
saveas(gcf, full_path);

% Plot 1: Magnet Heading before after correction (Town)
% Compute headings using magnetometer data and correct them using the calibration parameters.
% Calculate yaw angles before and after correction
heading_magnet_raw = atan2(-magtown_y, magtown_x);
heading_magnet = atan2(-corrected_town(:,2), corrected_town(:,1));

% Unwrap the Phase: MATLAB provides a function called unwrap which can be applied to 
% the yaw angle data to mitigate the phase wrapping issue. 
heading_magnet_raw = rad2deg(unwrap(heading_magnet_raw));
heading_magnet = rad2deg(unwrap(heading_magnet));

% Create a figure with the desired size
figure('Position', [100, 100, figWidthPixels, figHeightPixels]);
plot(heading_magnet_raw, 'r', 'DisplayName', 'Raw Yaw');
hold on;

% Remove offset
initial_yaw = heading_magnet(1);
heading_magnet = heading_magnet - initial_yaw;

plot(heading_magnet, 'b', 'DisplayName', 'Corrected Yaw');
xlabel('Time (samples)');
ylabel('Yaw Angle (degrees)');
title('Comparison of Raw and Corrected Yaw Angles From Magnetometer');
legend('show');
grid on;
full_path = fullfile(plot_path, 'plot_1_magnet_heading.png');
saveas(gcf, full_path);

% Plot 2: Gyro heading (yaw angle) using integrated gyro with initial unit of rad/s.
% Read the CSV file
imu_csv_filepath = '../data/town/town_imu.csv';
imu_data = readtable(imu_csv_filepath);

% Extract the timestamp and gyro data
time_stamps = imu_data.stamp;  % Extract the timestamp (already in seconds)

% Convert timestamps to elapsed time in seconds
% The first stamp is subtracted to start from t=0
time_seconds = time_stamps - time_stamps(1);

% Integrate the yaw rate (gyro_z) to get the yaw angle
heading_gyro = cumtrapz(time_seconds, imu_data.gyro_z);
heading_gyro_unwrapped = unwrap(heading_gyro);
heading_gyro = rad2deg(heading_gyro_unwrapped);

figure('Position', [100, 100, figWidthPixels, figHeightPixels]);
plot(time_seconds, heading_gyro);
xlabel('Time (s)');
ylabel('Yaw Angle (degrees)');
title('Integrated Yaw Angle from Gyro');
grid on;
full_path = fullfile(plot_path, 'plot_2_gyro_heading.png');
saveas(gcf, full_path);
imu_data.heading_magnet = heading_magnet;
imu_data.heading_gyro = heading_gyro;
writetable(imu_data, imu_csv_filepath);


% Plot 3) Filtered Heading.
plot_filtered_heading(imu_data, imu_csv_filepath, plot_path);


% Plot 4) Correct Accel data.
% Calculate the mean acceleration over the entire dataset.
mean_accel_x = mean(imu_data.accel_x);

% Subtract the mean acceleration from the acceleration data to correct it.
corrected_accel_x = imu_data.accel_x - mean_accel_x;

% Integrate the corrected acceleration to get the corrected velocity.
imu_velocity_x = cumtrapz(imu_data.stamp, corrected_accel_x);
imu_velocity_x = lowpass(imu_velocity_x, 0.5, 40);

% Plot the corrected velocity x
figure('Position', [100, 100, figWidthPixels, figHeightPixels]);
hold on;
plot(imu_data.stamp, imu_velocity_x, 'b', 'LineWidth', 1);
hold off;
xlabel('Time (s)');
ylabel('Velocity (m/s)');
title('Corrected Velocity X from IMU Data');
legend('corrected', 'Low-pass');
grid on;
full_path = fullfile(plot_path, 'plot_4_imu_velocity_x.png');
saveas(gcf, full_path);


% Plot 5) Compute velocity from GPS data.
imu_csv_filepath_gps = '../data/town/town_gps.csv';
gps_data = readtable(imu_csv_filepath_gps);

% Calculate time differences
time_diffs = diff(gps_data.stamp);

% Pre-allocate velocity array
gps_velocity = zeros(height(gps_data)-1, 1);

% Loop through GPS data to calculate displacements and velocity
for i = 1:(height(gps_data)-1)
    % Calculate displacement using the Haversine formula
    % Earth radius in meters.
    R = 6371000; 
    delta_lat = deg2rad(gps_data.latitude(i+1) - gps_data.latitude(i));
    delta_lon = deg2rad(gps_data.longitude(i+1) - gps_data.longitude(i));
    a = sin(delta_lat/2)^2 + cos(deg2rad(gps_data.latitude(i))) * cos(deg2rad(gps_data.latitude(i+1))) * sin(delta_lon/2)^2;
    c = 2 * atan2(sqrt(a), sqrt(1-a));
    d = R * c;
    
    % Calculate velocity (displacement over time)
    gps_velocity(i) = d / time_diffs(i);
end

figure('Position', [100, 100, figWidthPixels, figHeightPixels]);
plot(gps_data.stamp(2:end), gps_velocity, 'LineWidth', 1);
title('Velocity computed from GPS data');
xlabel('Time (s)');
ylabel('Velocity (m/s)');
grid on;
full_path = fullfile(plot_path, 'plot_5_gps_velocity.png');
saveas(gcf, full_path);


% Plot 6) Plot GPS position vs Position obtained from Dead-Reckoning.

% Step 0: Compare GPS displacement VS IMU displacement from forward velocity.

imu_displacement = cumtrapz(imu_data.stamp, imu_velocity_x)
gps_displacement = cumtrapz(gps_data.stamp(2:end), gps_velocity)
figure;
plot(imu_data.stamp, imu_displacement, 'r', 'DisplayName', 'IMU Displacement');
hold on;
plot(gps_data.stamp(2:end), gps_displacement, 'b', 'DisplayName', 'GPS Displacement');

hold off;
xlabel('Time (s)');
ylabel('Displacement (m)');
title('Comparison of IMU Vs GPS Displacement');
legend('show');
grid on;
full_path = fullfile(plot_path, 'plot_6_0_imu_vs_gps_displacement.png');
saveas(gcf, full_path);

% Step 1: Compare the product of integrated angular velocity and forward velocity from x_accel with y_accel
% Integrate x_accel directly from imu data to get x_velocity_sensor.
x_velocity_sensor = cumtrapz(imu_data.stamp, imu_data.accel_x)

% Compute angular velocity of heading (gyro_z) * x_velocity_sensor and compare it to imu_data.accel_y from the sensor.
angular_velocity_influence = imu_data.gyro_z .* x_velocity_sensor;
figure;

corrected_accel_y = butter_lowpass_filter(imu_data.accel_y, 0.8, 40, 1); % you might adjust the filter parameters as necessary
% Subtract the bias from the y_accel data
mean_bias_y = mean(corrected_accel_y);
corrected_accel_y = corrected_accel_y - mean_bias_y;
plot(imu_data.stamp, corrected_accel_y , 'r', 'DisplayName', 'imu sensor acceleration-y');

hold on;
plot(imu_data.stamp, angular_velocity_influence, 'b', 'DisplayName', 'angular velocity-z * imu velocity-x');

hold off;
xlabel('Time (s)');
ylabel('Acceleration (m/s^2)');
title('Comparison of Acceleration');
legend('show');
grid on;
full_path = fullfile(plot_path, 'plot_6_1_acceleration_y_comparison.png');
saveas(gcf, full_path);

% Step 2: Plot Trajectory
% get the heading angle in radian.
heading_angle_radians = deg2rad(imu_data.heading_magnet);

corrected_velocity_northing = cos(heading_angle_radians) .* imu_velocity_x;
corrected_velocity_easting = sin(heading_angle_radians) .* imu_velocity_x;

% Integrate the corrected velocities to get the displacement
displacement_easting = cumtrapz(imu_data.stamp, corrected_velocity_easting);
displacement_northing = cumtrapz(imu_data.stamp, corrected_velocity_northing);

% Plot the trajectory
figure;
plot(displacement_easting, displacement_northing, 'LineWidth', 1);
xlabel('Easting (m)');
ylabel('Northing (m)');
title('Corrected 2D Trajectory from IMU Data');
legend('Trajectory');
grid on;
full_path = fullfile(plot_path, 'plot_6_trajectory_imu.png');
saveas(gcf, full_path);

% Functions
function filtered_data = butter_lowpass_filter(data, cutoff, fs, order)
    % Design a low-pass Butterworth filter in SOS format
    [sos, g] = butter(order, cutoff / (fs / 2), 'low');

    % Apply the SOS filter to the data using zero-phase filtering
    filtered_data = filtfilt(sos, g, data);
end

function filtered_data = butter_highpass_filter(data, cutoff, fs, order)
    % Design a high-pass butterworth filter
    [b, a] = butter(order, cutoff/(fs/2), 'high');
    % Apply the filter to the data using zero-phase filtering
    filtered_data = filtfilt(b, a, data);
end

function imu_data = apply_complementary_filter(imu_data, alpha)
    imu_data.heading_complementary = imu_data.heading_gyro_high_filtered * alpha + (1 - alpha) * imu_data.heading_magnet_low_filtered;
end

function plot_filtered_heading(imu_data, imu_csv_filepath, PLOT_DIR)
    % Low-pass filter requirements
    low_order = 1;
    low_cutoff = 0.08;  % desired cutoff frequency of the filter, Hz
    low_fs = 40;        % sampling frequency

    % Apply the low-pass filter to the magnetometer data
    imu_data.heading_magnet_low_filtered = butter_lowpass_filter(imu_data.heading_magnet, low_cutoff, low_fs, low_order);

    % High-pass filter requirements
    high_order = 1;
    high_cutoff = 0.00001;  % desired cutoff frequency of the filter, Hz
    high_fs = 40;          % sampling frequency

    % Apply the high-pass filter to the gyro data
    imu_data.heading_gyro_high_filtered = butter_highpass_filter(imu_data.heading_gyro, high_cutoff, high_fs, high_order);

    % Use the complementary filter on your imu_data (adjust alpha as needed)
    alpha = 0.5;  % Example value, adjust based on your system and tests
    imu_data = apply_complementary_filter(imu_data, alpha);

    % Unwrap the complementary and IMU yaw data and convert to degree
    imu_data.heading_complementary = unwrap(deg2rad(imu_data.heading_complementary));
    imu_data.heading_complementary = rad2deg(imu_data.heading_complementary);

    imu_data.yaw_unwrapped = unwrap(deg2rad(imu_data.yaw));
    imu_data.yaw_unwrapped = rad2deg(imu_data.yaw_unwrapped);

    % Plot the raw and corrected yaw angles for comparison
    figWidthInches = 12;
    figHeightInches = 4;

    % Convert size to pixels (assuming 96 DPI)
    figWidthPixels = figWidthInches * 96;
    figHeightPixels = figHeightInches * 96;
    figure('Position', [100, 100, figWidthPixels, figHeightPixels]);
    hold on;
    plot(imu_data.stamp, imu_data.heading_gyro_high_filtered, 'r', 'LineWidth', 1);
    plot(imu_data.stamp, imu_data.heading_magnet_low_filtered, 'g', 'LineWidth', 1);
    plot(imu_data.stamp, imu_data.heading_complementary, 'm', 'LineWidth', 1);
    plot(imu_data.stamp, imu_data.yaw_unwrapped, 'b', 'LineWidth', 1);
    hold off;
    title('Gyro and Magnetometer Heading: High-pass vs Low-pass vs Complementary Filter');
    xlabel('Time Stamp');
    ylabel('Heading (degrees)');
    legend('High-pass Filtered Gyro Heading', 'Low-pass Filtered Magnet Heading', 'Complementary Filtered Heading', 'IMU yaw');
    grid on;
    plot_filename = fullfile(PLOT_DIR, 'plot_3_filtered_heading.png');
    saveas(gcf, plot_filename);
    writetable(imu_data, imu_csv_filepath);

end