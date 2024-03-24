% run this using: sudo matlab -softwareopengl
% Select a ROS bag file and topic
shape = 'circle';
addpath('~/Documents/MATLAB');

% Use sprintf to dynamically create the file path
filename = sprintf('../data/%s_imu/%s_imu.bag', shape, shape);
bag_select = rosbag(filename); % load the bag file
bSel = select(bag_select, 'Topic', '/imu'); % select the topic associated with IMU data

% Read messages from the selected bag file and topic
msg_struct = readMessages(bSel, 'DataFormat', 'struct');
disp(fieldnames(msg_struct{1}.MagField))

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
hold on % keep the current plot
scatter(magcircles_x, magcircles_y, 'r*') % scatter plot of the raw data

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

% Plot the corrected data and the fitted ellipse
scatter(circle_ellipse(:,1), circle_ellipse(:,2), 'b*'); % scatter plot of the corrected data
title('Soft Iron and Hard Iron Correction'); % title of the plot
legend('Before Correction', 'After Correction'); % legend of the plot
xlabel('Magnetic Field X (Gauss)'); % X-axis label
ylabel('Magnetic Field Y (Gauss)'); % Y-axis label



% Compute Heading using Yaw:
% the yaw angle typically represents the heading of a vehicle or object.
% Load the town data bag file
filename_town = '../data/town_imu/town_imu.bag';
bag_select_town = rosbag(filename_town); % load the bag file
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

% Calculate yaw angles before and after correction
yaw_raw = atan2(magtown_y, magtown_x);
yaw_corrected = atan2(corrected_town(:,2), corrected_town(:,1));

% Unwrap the Phase: MATLAB provides a function called unwrap which can be applied to 
% the yaw angle data to mitigate the phase wrapping issue. 
% It changes the jumps by adding multiples of +/- 360 degrees when it detects discontinuities greater than 180 degrees.
yaw_raw_unwrapped = unwrap(yaw_raw);
yaw_corrected_unwrapped = unwrap(yaw_corrected);

% Plot the raw and corrected yaw angles for comparison
figure;
plot(rad2deg(yaw_raw_unwrapped), 'r', 'DisplayName', 'Raw Yaw');
hold on;
plot(rad2deg(yaw_corrected_unwrapped), 'b', 'DisplayName', 'Corrected Yaw');
xlabel('Time (samples)');
ylabel('Yaw Angle (degrees)');
title('Comparison of Raw and Corrected Yaw Angles (Unwrapped)');
legend('show');
grid on;

