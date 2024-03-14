% Magnetometer Calibration and Plotting

% Shape variable: 'circle' or 'square'
shape = 'circle';

% Use sprintf to dynamically create the file path
filename = sprintf('../data/dead_reckoning_%s/dead_reckoning_%s.csv', shape, shape);


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
newFilename = sprintf('../data/dead_reckoning_%s/dead_reckoning_%s.csv', shape, shape);
writetable(magData, newFilename);

% Plot the original data and corrected.
figure;
scatter(magArray(:,1), magArray(:,2), 'r'); 
hold on;
scatter(xCorrected(:,1), xCorrected(:,2), 'g');
legend('Original', 'Corrected');
xlabel('magX (Gauss)');
ylabel('magY (Gauss)');
title(sprintf('Magnetometer Data Comparison: %s', shape)); 
saveas(gcf, sprintf('../data/plots/dead_reckoning_%s/magnetometer_before_after_corr_%s.png', shape, shape));


% % 2) Apply hard iron correction only
% xHardIronCorrected = magArray - b;  
% figure;
% scatter(magArray(:,1), magArray(:,2), 'r'); 
% hold on;scatter(xHardIronCorrected(:,1), xHardIronCorrected(:,2), 'b');  
% legend('Original', 'Hard Iron Corrected');
% xlabel('magX');
% ylabel('magY');
% title(sprintf('Hard Iron Correction: %s', shape));
% saveas(gcf, sprintf('../data/plots/dead_reckoning_%s/magnetometer_hard_iron_%s.png', shape, shape));
% 
% 
% % 3) Apply soft iron correction only
% xSoftIronCorrected = (magArray) * A;
% figure;
% scatter(magArray(:,1), magArray(:,2), 'r');  
% hold on;
% scatter(xSoftIronCorrected(:,1), xSoftIronCorrected(:,2), 'm'); 
% legend('Original', 'Soft Iron Corrected');
% xlabel('magX');
% ylabel('magY');
% title(sprintf('Soft Iron Correction: %s', shape));
% saveas(gcf, sprintf('../data/plots/dead_reckoning_%s/magnetometer_soft_iron_%s.png', shape, shape));
% 
