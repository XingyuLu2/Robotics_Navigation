% this part is for plotting the moving data
% ############################################

filename = 'bagdata_move_4.csv';
data = readtable(filename);
utm_east = data(:, 8);
utm_east = utm_east{:,:};
utm_north = data(:, 9);
utm_north = utm_north{:,:};

east_min = min(utm_east);
north_min = min(utm_north);
east_max = max(utm_east);
north_max = max(utm_north);
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
hold on;
xlabel('The UTM Easting Errors');
ylabel('The UTM Northing Errors');
title('Origin data subscribe to the mini data.');

% plot the lie fit the data points
num = size(utm_east_range);
c = polyfit(utm_east_range, utm_north_range,1);
y_f = polyval(c,utm_east_range);
plot(utm_east_range, y_f,'--r');
hold off;

% the error plot -- error between fit line and real data points
A = c(1);
B = -1;
C = c(2);
dis = [];
for i = 1 : num
    X = utm_east_range(i);
    Y = utm_north_range(i);
    d = abs(A*X + B*Y + C) / sqrt(A^2 + B^2);
    dis = [dis d];
end
subplot(2,1,2);
plot(dis, 'r');
xlabel('The i-th data point');
ylabel('The error values');
title('The error between fit line and real data points')
hold off;

V = var(dis);
M = mean(dis);
disp(M);
disp(V);
