%new
%%Mag Calibration process
Mag_data = readmatrix('mag.csv'); %transfer the data into matrix format
N1 = 320; % the first 4 circlr data range
N2 = 4600;

mag_x = Mag_data(N1:N2,5)*180/pi; %adjust the data
mag_y = Mag_data(N1:N2,6)*180/pi;

figure(1); 
subplot(1,3,1); %plot the orignial mag data
plot(mag_x,mag_y);
xlabel('mag_x');
ylabel('mag_y');
title("Raw data");
% calibration of hard-iron
a = (max(mag_x)+min(mag_x))/2; %find the offset of x and y axis
b = (max(mag_y)+min(mag_y))/2;
mag_x1 = mag_x-a;
mag_y1 = mag_y-b;

subplot(1,3,2); %plot the figure about calibration of hard iron
plot(mag_x1,mag_y1);
xlabel('mag_x');
ylabel('mag_y');
title("After calibration of hard iron");
% calibration of soft-iron
x1=-0.0006702;
y1=0.000733;
x2=0.001051;
y2=0.00005411;

r = sqrt((x1)^2+(y1)^2);
q = sqrt((x2)^2+(y2)^2);
h = asind(y1/r);
h1 = -h/180;
R = [cos(h1) sin(h1); -sin(h1) cos(h1)];
mag_x12 = cos(h1)*mag_x1 + sin(h1)*mag_y1;
mag_y12 = -sin(h1)*mag_x1 + cos(h1)*mag_y1;
s = q/r;
mag_x2 = mag_x12/s;
mag_y2 = mag_y12;

subplot(1,3,3); %plot the figure about calibration of soft iron
plot(mag_x2,mag_y2);
xlabel('mag_x');
ylabel('mag_y');
title("After calibration of soft iron");

% Integrated angular rate in z axis to get yaw
imu_data = readmatrix('imu.csv');
Gyro_z1 = imu_data(N1:N2,20)*180/pi;
Gyro_z = Gyro_z1*180/pi;
Ori_x = (imu_data(N1:N2,5)*180/pi)*180/pi;

seq = N2 - N1;
ed = 0.02503*seq;
t = linspace(0,ed,seq+1); % Time

yaw0 = Ori_x; % -Ori_x; % the yaw read from imu directly
yaw01 = unwrap(Ori_x,2*pi);
yaw1 = (atan2(mag_y2,mag_x2))*180/pi; % the yaw from mag   
yaw11 = unwrap(yaw1,2*pi); % accumulate the yaw from mag
yaw2 = cumtrapz(t,Gyro_z); % yaw from integrated angular rate

figure(2);
subplot(2,1,1);
plot(yaw0);
xlabel('Time');
ylabel('Yaw');
title("3.Yaw read from original Imu data");
subplot(2,1,2);
plot(yaw1);
xlabel('Time');
ylabel('Yaw');
title("1.Yaw calculated by mag");

figure(3);
subplot(3,1,1);
plot(yaw01);
xlabel('Time');
ylabel('Yaw');
title("3.Yaw accumulated from original yaw data of IMU");
subplot(3,1,2);
plot(yaw11);
xlabel('Time');
ylabel('Yaw');
title("2.Yaw accumulated from mag data");
subplot(3,1,3);
plot(yaw2);
xlabel('Time');
ylabel('Yaw');
title("4.Yaw integrated by Gyro_z");

yaw10 = unwrap((atan2(mag_y,mag_x))*180/pi,2*pi);
yaw12 = unwrap((atan2(mag_y2,mag_x2))*180/pi,2*pi);
figure(4);
subplot(2,1,1);
plot(yaw10);
xlabel('Time');
ylabel('Yaw');
title("1.Yaw calculated by raw mag data");
subplot(2,1,2);
plot(yaw12);
xlabel('Time');
ylabel('Yaw');
title("2.Yaw calculated by adjusted mag data");

% filter mag data
a1 = 0.95;
yaw_filter = a1*yaw2 + (1-a1)*yaw0;
figure(5);
% subplot(2,1,1);
plot(yaw_filter);
xlabel('Time');
ylabel('Yaw');
title("Filted Yaw");


%
% The velocity calculated by GPS data
Gps_data = readmatrix('gps.csv'); %transfer the data into matrix format
No1 = 120;      % range(120,220,350,turnR 360,R470,L520,L550,L630,L740,L860,910)
No2 = 910;
lat = Gps_data(No1:No2,5);
lon = Gps_data(No1:No2,6);

v = [];
for i = 1:(No2-No1)
    R = 6371.004; % The average radius of the Earth (km)
    
    LatA = 90-lat(i);
    LatB = 90-lat(i+1);
    LonA = lon(i);
    LonB = lon(i+1);
    
    C = sin(LatA)*sin(LatB)*cos(LonA-LonB) + cos(LatA)*cos(LatB);
    D = R*(acos(C))*pi/180; %Distance ,(km)
    t = 0.99857;% Time(s)
    
    v(end+1)= 1000*D/t; %(m/s)
 
end

figure(6);
subplot(3,1,1);
plot(v);
xlabel('Time');
ylabel('Velocity_x');
title("The velocity calculated by GPS data in x-axis")

% The velocity calculated by IMU data
IMU_data = readmatrix('imu.csv'); %transfer the data into matrix format
No3 = 40*No1;   % The first straight line of the whole route;
No4 = 40*No2;
Accel_x = IMU_data(No3:No4,30)*180/pi; % m/s^2

seq = No4 - No3;
ed = 0.02503*seq;
t1 = linspace(0,ed,seq+1);  %Time (s) 
Velo_x = cumtrapz(t1,Accel_x); % (m/s)

subplot(3,1,2);
plot(Velo_x);
xlabel('Time');
ylabel('Velocity_x');
title("The velocity calculated by IMU data in x-axis")

% Adjust Imu_x data
Saccel_x = IMU_data(42870:47470,30)*180/pi;
mean_accelx = mean(Saccel_x) ;
AdAccel_x = Accel_x - mean_accelx; 

Velo_x1 = cumtrapz(t1,AdAccel_x); % m/s   x'
subplot(3,1,3);
plot(Velo_x1);
xlabel('Time');
ylabel('Velocity_x');
title("The adjusted velocity calculated by IMU data in x-axis")

% Dead Reckoning 1
Accel_y = IMU_data(No3:No4,31)*180/pi; % m/s^2 y''
Ang_z = IMU_data(No3:No4,20);%*180/pi; % rad/s  omega    ?????
Sang_z = IMU_data(42870:47470,20);%*180/pi;   ??????
mean_angz = mean(Sang_z) ;
Adang_z = Ang_z - mean_angz; 


y1 = Adang_z .* Velo_x ; 

% figure(7);
% subplot(2,1,1);
% plot(y1);
% xlabel('Time');
% ylabel(" value ");
% title(" Omega mutiple X' ")
% subplot(2,1,2);
% hold on
% plot(Accel_y,'r');
% xlabel('Time');
% ylabel('value');
% title(" accel_y of imu ")

figure(7);
plot(y1);
hold on
plot(Accel_y,'r');

% adjust the whole yaw 
mag_x0 = Mag_data(No3:No4,5)*180/pi; 
mag_y0 = Mag_data(No3:No4,6)*180/pi;

mag_x01 = mag_x0-a;
mag_y01 = mag_y0-b;
mag_x02 = cos(h1)*mag_x01 + sin(h1)*mag_y01;
mag_y02 = -sin(h1)*mag_x01 + cos(h1)*mag_y01;
mag_xv = mag_x02/s;
mag_yv = mag_y02;
yaw_v = (atan2(mag_yv,mag_xv))*180/pi;

Vx = Velo_x1.*cos(yaw_v);
Vy = Velo_x1.*sin(yaw_v);

Sx = cumtrapz(t1,Vx);
Sy = cumtrapz(t1,Vy);

figure(8);
%plot(yaw_v);
%plot(Sx,Sy);
plot(Accel_x);




