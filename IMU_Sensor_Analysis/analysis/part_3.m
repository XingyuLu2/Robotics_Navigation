% the approximate time:
% Citation : https://www.mathworks.com/matlabcentral/fileexchange/38812-latlon-distance
% which is matlab function for computing real distance via latitude and
% Longtitude
% whole driving time(left first circle) : 727 seconds
% circle time : 100 sconds

% 1. Estimate the Heading(yaw)
% 1.1 Magnetomeer Calibration
% ###################################
circle_start = 552;
circle_end = 4541;
mag_filename = 'mag.csv';
data = readtable(mag_filename);
% get the points about driving in circles
mag_X = data(:, 5); 
mag_X = mag_X{:,:};
mag_Y = data(:, 6);
mag_Y = mag_Y{:,:};

% plot the Mag readings before correction
subplot(1,3,1);
plot(mag_X(circle_start:circle_end), mag_Y(circle_start:circle_end), '--r');
xlabel('The magnetometer X');
ylabel('The magnetometer Y');
title('Graph Before Correction');

% start the Hard-Iron Correction
X_min = min(mag_X(circle_start:circle_end));
X_max = max(mag_X(circle_start:circle_end));
Y_min = min(mag_Y(circle_start:circle_end));
Y_max = max(mag_Y(circle_start:circle_end));
alpha = (X_max + X_min) / 2;
bet = (Y_max + Y_min) / 2;
mag_X = mag_X - alpha;
mag_Y = mag_Y - bet;

subplot(1,3,2);
plot(mag_X(circle_start:circle_end), mag_Y(circle_start:circle_end), '--r');
xlabel('The magnetometer X');
ylabel('The magnetometer Y');
title('Graph After Hard Iron');

% start the correction of Soft-Iron
x1 = 0.001085; y1 = -0.0002679; % Mijor Axis R
x2 = 9.861e-05; y2 = 0.0005559; % Minor Axis Q
q = sqrt(x2*x2 + y2*y2);
r = sqrt(x1*x1 + y1*y1);
theta = asin(y1/r);
% the rotation computation
mag_X = cos(theta)*mag_X + sin(theta)*mag_Y;
mag_Y = -sin(theta)*mag_X + cos(theta)*mag_Y;
% produce a approximate circle
scale_fac = q / r;
mag_X = mag_X / scale_fac;

subplot(1,3,3);
plot(mag_X(circle_start:circle_end), mag_Y(circle_start:circle_end), '--r');
xlabel('The magnetometer X');
ylabel('The magnetometer Y');
title('Graph After Hard Soft Iron');
figure;
% ################################################

% 1.2. Calculate the Yaw Angle from magnetometer
% ############################################
% the Yaw angle using magnetometer
time_circle = 100; % 1 / 40 * (4541 - 552)
X = linspace(0,910-117,36261-4542+1);

ang = -atan2(mag_Y(4542:36261), mag_X(4542:36261));
plot(X, ang, 'r');
xlabel('the i-th data point');
ylabel('the angular rate value');
hold on;

% the Yaw angle using angular rate
data_imu = readtable('imu.csv');
ang_Z = data_imu(4542:36261, 20);
ang_Z = ang_Z{:,:}; ang_Z = ang_Z * 180 / pi;


y_Int = cumtrapz(X, ang_Z);
plot(X, y_Int, 'b');
xlabel('the time series');
ylabel('The integrated Yaw using Gyro:');
title('The Yaw Integrated from Mag and from Gyro');
legend({'yaw from magnetometer', 'yaw from Gyro'})
hold off;
figure;
% ##############################################

% 1.3. Use a complementary filter to combine measurements
%      from the magnetometer and yaw rate
% ##############################################
hpf = 0.99;
lpf = 0.01;
thetaZ = lpf * ang + hpf * y_Int;

plot(X, thetaZ, 'r');
xlabel('the i-th data point');
ylabel('the yaw value'); hold on;

data_i = readtable('imu_raw.csv');
yaw_imu = data_i(4542:36261, 3);
yaw_imu = yaw_imu{:,:}; yaw_imu = yaw_imu * pi / 180;
yaw_imu = unwrap(yaw_imu);
plot(X, yaw_imu, 'b');
xlabel('the time seris');
ylabel('the yaw value');
title('The Yaw from Compelmentary filter and IMU sensor'); 
legend({'Yaw from Complementary filter', 'Yaw from IMU'});hold off;
figure;
% ##############################################

% 2. Esimate the forward velocity
% 2.1. Integrate the forward acc to estimate the forward velocity
% ##########################################
num = 47470; drive_stop = 40130;
% strai_end = 40130 - 1297*4
straight_start = 4542; straight_end = 36261;
acc_X = data_imu(straight_start:straight_end, 30);
acc_X = acc_X{:,:};acc_X = acc_X * 180 / pi;

time_whole = 1187 - 1; % first 3.5 circles plus the rest
time_straight = 910 - 117;
X = linspace(1,time_straight,straight_end - straight_start + 1);
V_X_raw = cumtrapz(X, acc_X);
subplot(3,1,1);
plot(X,acc_X, '--r');
xlabel('the time series (second)');
ylabel('the velocity value (m/s)');
title('Velocity about X axis integrated by acc');
% ###############################################

% 2.2. Calculate an estimate of velocity from GPS
% ###############################################
% the GPS straight driving is at 117-910, actually stop a 1005
data_gps = readtable('gps.csv');
lati = data_gps(1:1005, 5);
lati = lati{:,:};
longti = data_gps(1:1005, 6);
longti = longti{:,:};
dis = [];
for i = 1 : 1004
  latlon1 = [lati(i) longti(i)];
  latlon2 = [lati(i+1) longti(i+1)];
  % convert km to meter
  dis = [dis lldistkm(latlon1, latlon2)];
end
dis = dis*1000; % convert from km to m
V_gps = dis / 1;

subplot(3,1,2)
plot(V_gps(117:910), '--r');
xlabel('the time series (second)');
ylabel('the GPS velocity value (m/s)');
title('the velocity computed by GPS measurements');
% ###############################################

% the integrated velocity does not make sense
% 2.3. Make adjustments to Acc measurement
% ###############################################
acc_X = data_imu(straight_start:straight_end, 30);
acc_X = acc_X{:,:}; acc_X = acc_X * 180 / pi;

bias_acc = data_imu(42870:47470, 30);
bias_acc = bias_acc{:,:}; bias_acc = bias_acc * 180 / pi;
bias_acc = mean(bias_acc);
acc_X = acc_X - bias_acc;

acc_X = smoothdata(acc_X);
%{
delt = 0.001;
count_bias = 0; count_whole = 0; f = 0;
for i =  1 : straight_end - straight_start
    d = abs(acc_X(i+1) - acc_X(i));
    if(d <= delt)
        if(count_bias ~= 0 && f == 1)
            bias_acc = mean(acc_X(i-count_whole:i-count_whole+count_bias));
            acc_X(i-count_whole:i-1) = acc_X(i-count_whole:i-1) - bias_acc;
            count_bias = 0; count_whole = 0; f= 0;
        end
        count_bias = count_bias + 1;
        count_whole = count_whole + 1;
    end
    if(d > delt)
        count_whole = count_whole + 1;
        f = 1;
    end
end
%}
X = linspace(1,time_straight,straight_end - straight_start + 1);
V_X = cumtrapz(X, acc_X);

subplot(3,1,3);
plot(X, V_X, '--r');
xlabel('the time series (second)');
ylabel('the velocity value (m/s)');
title('Velocity by Adjusted Acc Measurement');
figure;
% ###############################################

% 3. Dead Reckoning with IMU
% ############################################################
% Integrate IMU to obtain displacement adn compare with GPS
% circle end = 4541, car stop = 40130
acc_Y = data_imu(straight_start:straight_end, 31);
acc_Y = acc_Y{:,:};acc_Y = acc_Y * 180 / pi;
plot(X, acc_Y, 'r');
hold on;

ang_Z = data_imu(straight_start:straight_end, 20); 
ang_Z = ang_Z{:,:};ang_Z = ang_Z * (180 / pi);
bias_ang = data_imu(42870:47470, 20); 
bias_ang = bias_ang{:,:};bias_ang = bias_ang * (180 / pi);
bias_ang = mean(bias_ang);
ang_Z = ang_Z - bias_ang;
w_V_X = ang_Z .* V_X;
plot(X, w_V_X, 'b');
xlabel('the time series (second)');
ylabel('the Acc value (m/s)');
title('w*V vs y_{obs}');
legend({'y_{obs}', 'w * V'});
hold off; 
figure;

% Integrate to estimate the trajactory
ang = atan2(mag_Y(straight_start:straight_end), mag_X(straight_start:straight_end));
T = linspace(1,time_straight,straight_end - straight_start + 1);
V_X = cumtrapz(T, acc_X);
Xn = cumtrapz(T, V_X .* cos(ang));
Xe = cumtrapz(T, V_X .* sin(ang));

% the trajactory in GPS
filename = 'gps.csv';
data_gps = readtable(filename);
utm_east = data_gps(117:910, 8);
utm_east = utm_east{:,:};
utm_north = data_gps(117:910, 9);
utm_north = utm_north{:,:};
east_min = min(utm_east);
north_min = min(utm_north);
utm_east_range = utm_east - east_min;
utm_north_range = utm_north - north_min;

pos = Xn > 0; neg = Xn < 0;
Xn(pos) = -Xn(pos); Xn(neg) = -Xn(neg);
pos = Xe > 400; neg = Xe < 400;
Xe(pos) = Xe(pos) - 400; Xe(neg) = Xe(neg) + 400;
d_x = min(Xe) - utm_east_range(1); d_y = Xe==min(Xe);
d_y = Xn(d_y) - utm_north_range(1);
Xe = Xe - d_x; Xn = Xn - d_y;
plot(Xe, Xn,'--b');
hold on;
plot(utm_east_range, utm_north_range, '--r');
title('The Estimated Trajactory vs GPS Trajactory');
legend({' Estimated Trajactory', 'GPS Trajactory'});
figure;

% Estimation of X_c
X = linspace(1,time_straight,straight_end - straight_start + 1);
V_X = cumtrapz(X, acc_X);
acc_X_obs = data_imu(straight_start:straight_end, 30);
acc_X_obs = acc_X_obs{:,:}; acc_X_obs = acc_X_obs * 180 / pi;
% X = linspace(1,time_straight,straight_end - straight_start + 1);
% V_X_obs = cumtrapz(X, acc_X_obs); + ang_Z.*V_X_obs 

X_c_1 = acc_X_obs - acc_X - ang_Z .* V_X;
w_dot = ang_Z / (time_straight / (straight_end - straight_start));
X_c_1 = X_c_1 ./ (w_dot + ang_Z .* ang_Z);
plot(X_c_1, '--b');title('The Estimate X_c at each time point');
% ####################################################
%}