% this part is for plotting and analyzing 
% the stationary data
% ############################################
% the first circle : 10 : 36
% the forst half circle : 10 : 26
% the whole circle driving route : 1 : 116
% the straight line: 117:910
% filename = 'part2_trajectory_gps_out.csv';
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

plot(utm_east, utm_north, '--r');
xlabel('The UTM Easting Errors');
ylabel('The UTM Northing Errors');
title('Origin data subscribe to the mini data.');
