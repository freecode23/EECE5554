% Magnetometer Calibration and Plotting

% Shape variable: 'circle' or 'town'
shape = 'circle';

% Use sprintf to dynamically create the file path
filename = sprintf('../data/%s_imu/%s_imu.csv', shape, shape);


% Read the entire magnetometer data from the CSV file without selecting specific columns
magData = readtable(filename);

% Extract only the relevant magnetometer data as an array for calibration
magArray = table2array(magData(:, {'mag_x', 'mag_y', 'mag_z'}));

% Perform calibration using magcal
[A, b, expMFS] = magcal(magArray);

% 1) Apply the calibration to the magnetometer data
xCorrected = (magArray - b) * A;

% Add the corrected data as new columns to the magData table
magData.mag_x_corr = xCorrected(:, 1);
magData.mag_y_corr = xCorrected(:, 2);
magData.mag_z_corr = xCorrected(:, 3);

% Save the updated table to a new CSV file
newFilename = sprintf('../data/%s_imu/%s_imu.csv', shape, shape);
writetable(magData, newFilename);

% Plot the original data and corrected.
figure;
scatter(magArray(:,1), magArray(:,2), 'r', 'filled'); 
hold on;
scatter(xCorrected(:,1), xCorrected(:,2), 'b', 'filled'); 
legend('Before Calibration', 'After Calibration');
xlabel('mag X (Gauss)');
ylabel('mag Y (Gauss)');
title(sprintf('Magnetometer Before VS After: %s', shape)); 
saveas(gcf, sprintf('../data/plots/%s_imu/mag_field_before_after_calib.png', shape, shape));

