% Comvert the image from RGB type 
% to the Gray type, because the raw images taken
% are in the RGB type, them resize all of them

% for RGB-to-Gray Coverting 
string1 = "Image"; 

% resize with respect of image_raw1.jpg
im = imread('Image1.jpg');
img_gray = rgb2gray(im);
[columns, rows] = size(img_gray);
% from 20 million pixels to about 1 million pixels
rate_resize = 5000000 / (rows*columns);

for i = 1:20
    str = string1 + i + ".jpg";
    img_rgb = imread(str);
    % convert the img to gray
    img_gray = rgb2gray(img_rgb);
    % resize the images
    im_i = imresize(img_gray, rate_resize);
    disp(size(im_i));
    % for showing each resized gray image
    imshow(im_i);
    imwrite(im_i, "image"+i+".jpg");
end

