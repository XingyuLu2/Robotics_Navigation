data_mag = readtable('mag.csv');

mag_X = data_mag(4542:25000, 5);
mag_X = mag_X{:,:};
mag_Y = data_mag(4542:25000, 6);
mag_Y = mag_Y{:,:};

plot(mag_X, mag_Y, '--r');