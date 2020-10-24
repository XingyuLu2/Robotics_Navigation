% this part is for plotting and analyzing 
% the stationary data
% ############################################

filename = 'bagdata_whole_4_100.csv';
data = readtable(filename);
utm_east = data(:, 8);
utm_east = utm_east{:,:};
utm_north = data(:, 9);
utm_north = utm_north{:,:};

east_min = min(utm_east);
north_min = min(utm_north);
east_mean = mean(utm_east);
north_mean = mean(utm_north);
disp('The mean value of utm easting:')
disp(east_mean);
disp('The mean value of utm northing:')
disp(north_mean);
utm_east_range = utm_east - east_min;
utm_north_range = utm_north - north_min;
mean_e = east_mean - east_min;
mean_n = north_mean - north_min;

subplot(2,1,1);
plot(mean_e,mean_n,'Or');
hold on;
plot(utm_east_range, utm_north_range, '*k');
xlabel('The UTM Easting Errors');
ylabel('The UTM Northing Errors');
title('Origin data subscribe to the mini data.');
hold on;

subplot(2,1,2);
data_num = size(utm_east);
mean_east_plot = [];
mean_north_plot = [];
for i = 1 : data_num
    c = mean(utm_east_range(1:i));
    mean_east_plot = [mean_east_plot;c];
    c = mean(utm_north_range(1:i));
    mean_north_plot = [mean_north_plot;c];
end
plot(mean_east_plot, '--r');
hold on;
plot(mean_north_plot, '--b');
xlabel('The number of data');
ylabel('The mean values');
title('Mean value change according to time');
hold off;

V_n = var(utm_north_range);
V = var([utm_east_range utm_north_range]);
M = mean([utm_east_range utm_north_range]);
disp(M);
disp(V);
disp(V_n);

