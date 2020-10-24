% this part is for plotting and analyzing 
% the stationary data
% ############################################

filename = 'stationary_clear_data.csv';
data = readtable(filename);
utm_east = data(:, 8);
utm_east = utm_east{:,:};
utm_north = data(:, 9);
utm_north = utm_north{:,:};

east_min = min(utm_east);
north_min = min(utm_north);
east_mean = mean(utm_east);
north_mean = mean(utm_north);
disp('The mean original value of utm easting:')
disp(east_mean);
disp('The mean original value of utm northing:')
disp(north_mean);
utm_east_range = utm_east - east_min;
utm_north_range = utm_north - north_min;
mean_e = east_mean - east_min;
mean_n = north_mean - north_min;

subplot(3,1,1);
plot(mean_e,mean_n,'Ob');
hold on;
plot(utm_east_range, utm_north_range, '*r');
xlabel('The UTM Easting values');
ylabel('The UTM Northing values');
title('UTM Values Distribution with respect to Mini Data Point.');

subplot(3,1,2);
east_hist = histogram(utm_east_range);
xlabel('The UTM easting values');
ylabel('The number of data points');
title('The UTM easting distribution');

subplot(3,1,3);
north_hist = histogram(utm_north_range);
xlabel('The UTM northing values');
ylabel('The number of data points');
title('The UTM northing distribution');
figure;
Std = std([utm_east_range utm_north_range]);
M = mean([utm_east_range utm_north_range]);
disp('The mean values : ');
disp(M);
disp('The standard deviation values : ');
disp(Std);

% analysis for the UTM easting and northing noise
utm_east_noise = utm_east_range - mean_e;
utm_north_noise = utm_north_range - mean_n;

subplot(2,1,1);
east_hist = histogram(utm_east_noise, 100);
xlabel('The UTM easting  noise values');
ylabel('The number of data points');
title('The UTM easting noise distribution');

subplot(2,1,2);
north_hist = histogram(utm_north_noise, 100);
xlabel('The UTM northing noise values');
ylabel('The number of data points');
title('The UTM northing noise distribution');
figure;

Std = std([utm_east_noise utm_north_noise]);
M = mean([utm_east_noise utm_north_noise]);
disp('The mean noises : ');
disp(M);
disp('The standard deviation noises : ');
disp(Std);

% Visualize the Fix Quality
fix_quality = data(:, 12);
fix_quality = fix_quality{:,:};
histogram(fix_quality);
xlabel('the i-th GPS data point');
ylabel('the value of fix quality');
title('Fix Quality Distribution');

