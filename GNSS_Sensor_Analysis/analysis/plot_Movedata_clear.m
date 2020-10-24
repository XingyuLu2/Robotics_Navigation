% this part is for plotting the moving data
% ############################################

filename = 'move_clear_data.csv';
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
xlabel('The UTM Easting');
ylabel('The UTM Northing');
title('The Moving Route(subscribe to the mini data)');
figure;


% For Computing & Visualizing the noise along trajectory 
start_point = [utm_east_range(1) utm_north_range(1)];
% the data point at which we took a turn
turns = [0.01333 6.908; 6.715 0.00611; 21.45 16.41; 14.56 22.72];
idx_begin = 1; dis = [];
for i = 1:4
    idx_t = knnsearch(utm_matrix, turns(i,:), 'K', 1);
    num = size(utm_east_range(idx_begin:idx_t));
    c = polyfit(utm_east_range(idx_begin:idx_t), utm_north_range(idx_begin:idx_t),1);
    y_f = polyval(c,utm_east_range(idx_begin:idx_t));
    A = c(1);
    B = -1;
    C = c(2);
    
    for j = idx_begin : idx_t
        X = utm_east_range(j);
        Y = utm_north_range(j);
        d = abs(A*X + B*Y + C) / sqrt(A^2 + B^2);
        dis = [dis d];
    end
    idx_begin = idx_t+1;
end
plot(dis);
xlabel('the time step during trahectory');
ylabel('the error value');
title('The error values according to time');
figure;

histogram(dis);
xlabel('the error value');
ylabel('the number of data');
title('The Error Value Distribution');
figure;

% View the Fix Quality
Fix_Quality = data(:, 12);
Fix_Quality = Fix_Quality{:,:};
histogram(Fix_Quality);
xlabel('the Fix Quality Value');
ylabel('the number of data');
title('The Fix Quality Distribution');

