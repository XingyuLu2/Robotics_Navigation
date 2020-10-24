% this part is for plotting the moving data
% ############################################

filename = 'move_reflect_data.csv';
data = readtable(filename);
utm_east = data(:, 8);
utm_east = utm_east{:,:};
utm_north = data(:, 9);
utm_north = utm_north{:,:};

east_min = min(utm_east);
north_min = min(utm_north);
east_max = max(utm_east);
north_max = max(utm_north);

utm_east_range = utm_east - east_min;
utm_north_range = utm_north - north_min;
utm_matrix = [utm_east_range utm_north_range];
plot(utm_east_range, utm_north_range, '*r');
xlabel('The UTM Easting Errors');
ylabel('The UTM Northing Errors');
title('The Moving Route(subscribe to the mini data)');
hold on;

% Drawing approximated fit lines 
% to get Estimated Real Trahjectory

dis = []; % for storing the error values
start_end_x = [0.5949 16.2]; start_end_y = [11.66 0.1288];
plot(start_end_x, start_end_y, 'k'); hold on
% Compute the error along Trajectory
idx_t = knnsearch(utm_matrix, [0.5949 11.66], 'K', 1);
A = (11.66-0.1288) / (0.5949-16.2);
B = -1;
C = 16.2 - A*0.5949;
for j = 1 : idx_t
    X = utm_east_range(j);
    Y = utm_north_range(j);
    d = abs(A*X + B*Y + C) / sqrt(A^2 + B^2);
    dis = [dis d];
end

start_point = [0.5949 11.66]; end_point = [8.613 21.03];
idx_begin = knnsearch(utm_matrix, start_point, 'K', 1);
idx_t = knnsearch(utm_matrix, end_point, 'K', 1);
num = size(utm_east_range(idx_begin:idx_t));
c = polyfit(utm_east_range(idx_begin:idx_t), utm_north_range(idx_begin:idx_t),1);
y_f = polyval(c,utm_east_range(idx_begin:idx_t));
plot(utm_east_range(idx_begin:idx_t), y_f, 'k'); hold on;
% Compute the error along Trajectory
A = c(1);
B = -1;
C = c(2);
for j = idx_begin : idx_t
    X = utm_east_range(j);
    Y = utm_north_range(j);
    d = abs(A*X + B*Y + C) / sqrt(A^2 + B^2);
    dis = [dis d];
end

start_end_x = [8.613 26.18]; start_end_y = [21.03 9.975];
plot(start_end_x, start_end_y, 'k'); hold on;
% Compute the error along Trajectory
idx_begin = knnsearch(utm_matrix, [8.613 21.03], 'K', 1);
idx_t = knnsearch(utm_matrix, [26.18 9.975], 'K', 1);
A = (21.03-9.975) / (8.613-26.18);
B = -1;
C = 26.18 - A*8.613;
for j = idx_begin : idx_t
    X = utm_east_range(j);
    Y = utm_north_range(j);
    d = abs(A*X + B*Y + C) / sqrt(A^2 + B^2);
    dis = [dis d];
end

start_point = [26.18 9.975]; end_point = [16.2 0.1288];
idx_begin = knnsearch(utm_matrix, start_point, 'K', 1);
idx_t = knnsearch(utm_matrix, end_point, 'K', 1);
num = size(utm_east_range(idx_begin:idx_t));
c = polyfit(utm_east_range(idx_begin:idx_t), utm_north_range(idx_begin:idx_t),1);
y_f = polyval(c,utm_east_range(idx_begin:idx_t));
plot(utm_east_range(idx_begin:idx_t), y_f, 'k');
figure;
% Compute the error along Trajectory
A = c(1);
B = -1;
C = c(2);
for j = idx_begin : idx_t
    X = utm_east_range(j);
    Y = utm_north_range(j);
    d = abs(A*X + B*Y + C) / sqrt(A^2 + B^2);
    dis = [dis d];
end

idx_begin = knnsearch(utm_matrix, [16.2 0.1288], 'K', 1);
A = (11.66-0.1288) / (0.5949-16.2);
B = -1;
C = 16.2 - A*0.5949;
for j = idx_begin : size(utm_matrix, 1)
    X = utm_east_range(j);
    Y = utm_north_range(j);
    d = abs(A*X + B*Y + C) / sqrt(A^2 + B^2);
    dis = [dis d];
end

plot(dis);
xlabel('The time steps');
ylabel('The Error values');
title('The Error values according to time steps');figure;
histogram(dis);
xlabel('The Error Values');
ylabel('The Number od data');
title('The Error Distribution');

% View the Fix Quality
% View the Fix Quality
Fix_Quality = data(:, 12);
Fix_Quality = Fix_Quality{:,:};
histogram(Fix_Quality);
xlabel('the Fix Quality Value');
ylabel('the number of data');
title('The Fix Quality Distribution');