% this part is for plotting and analyzing 
% the stationary data
% ##########################################
mag_file = 'mag_stationary.csv';
imu_file = 'imu_stationary.csv';
% mag_file = 'part2_trajectory_mag_out.csv';
% imu_file = 'part2_trajectory_imu_out.csv';
data_mag = readtable(mag_file);
data_imu = readtable(imu_file);

% the 3 angular retes
ang_X = data_imu(1:3800, 18);
ang_X = ang_X{:,:};
ang_Y = data_imu(1:3800, 19);
ang_Y = ang_Y{:,:};
ang_Z = data_imu(1:3800, 20);
ang_Z = ang_Z{:,:};

% the 3 accelerators
acc_X = data_imu(1:3800, 30);
acc_X = acc_X{:,:};
acc_Y = data_imu(1:3800, 31);
acc_Y = acc_Y{:,:};
acc_Z = data_imu(1:3800, 32);
acc_Z = acc_Z{:,:};

% the 3 magnetometers
mag_X = data_mag(1:3800, 5);
mag_X = mag_X{:,:};
mag_Y = data_mag(1:3800, 6);
mag_Y = mag_Y{:,:};
mag_Z = data_mag(1:3800, 7);
mag_Z = mag_Z{:,:};

% Plotting the X Y Z values respectively
subplot(3,2,1);
plot(mag_X, '.b');
xlabel('The i-th data point');
ylabel('The Magnetometer data');
title('The Magnetometer X plotting');
subplot(3,2,3);
plot(mag_Y, '.b');
xlabel('The i-th data point');
ylabel('The Magnetometer data');
title('The Magnetometer Y plotting');
subplot(3,2,5);
plot(mag_Z, '.b');
xlabel('The i-th data point');
ylabel('The Magnetometer data');
title('The Magnetometer Z plotting');

subplot(3,2,2);
histogram(mag_X);
xlabel('The i-th data point');
ylabel('The num of data');
title('The Magnetometer X plotting');
subplot(3,2,4);
histogram(mag_Y);
xlabel('The i-th data point');
ylabel('The num of data');
title('The Magnetometer Y plotting');
subplot(3,2,6);
histogram(mag_Z);
xlabel('The i-th data point');
ylabel('The num of data');
title('The Magnetometer Z plotting');

