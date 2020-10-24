% Load images.'image1.jpg';
%{
buildingDir = fullfile({'image1.jpg';'image2.jpg';'image3.jpg';'image4.jpg';'image5.jpg';'image6.jpg';'image7.jpg';'image8.jpg';'image9.jpg';'image10.jpg';'image11.jpg';'image12.jpg';'image13.jpg';'image14.jpg';'image15.jpg';'image16.jpg';'image17.jpg';'image18.jpg';'image19.jpg';'image20.jpg'});
buildingScene = imageDatastore(buildingDir);
montage(buildingScene.Files, 'Size', [4 5]);
%}

buildingDir = fullfile({'image1.jpg';'image2.jpg';'image3.jpg';'image4.jpg';'image5.jpg'});
% buildingDir = fullfile({'image1_rect.jpg';'image2_rect.jpg';'image3_rect.jpg';'image4_rect.jpg';'image5_rect.jpg'});
buildingScene = imageDatastore(buildingDir);

% Display images to be stitched
montage(buildingScene.Files, 'Size', [1 6]);

% Read the first image from the image set.
I = readimage(buildingScene, 1);

% Initialize features for I(1)
% grayImage = rgb2gray(I);
grayImage = I;
% using harris corner detector to find the features
[y, x, m] = harris(grayImage,1000,'tile',[2 2], 'disp');
points = [x y];
[features, points] = extractFeatures(grayImage, points);

% Initialize all the transforms to the identity matrix. Note that the
% projective transform is used here because the building images are fairly
% close to the camera. Had the scene been captured from a further distance,
% an affine transform would suffice.
numImages = numel(buildingScene.Files);
tforms(numImages) = projective2d(eye(3));

% Initialize variable to hold image sizes.
imageSize = zeros(numImages,2);

% Iterate over remaining image pairs
for n = 2:numImages
    
    % Store points and features for I(n-1).
    pointsPrevious = points;
    featuresPrevious = features;
        
    % Read I(n).
    I = readimage(buildingScene, n);
    
    % Convert image to grayscale.
    grayImage = I;    
    % grayImage = rgb2gray(I);
    % Save image size.
    imageSize(n,:) = size(grayImage);
    
    % Detect and extract harris corner features for I(n).
    % using the harris corner detector to find the features
    [y, x, m] = harris(grayImage,1000,'tile',[2 2],'disp');
    % covert data type into cornerPoints
    points = [x y];
    [features, points] = extractFeatures(grayImage, points);
  
    % Find correspondences between I(n) and I(n-1).
    indexPairs = matchFeatures(features, featuresPrevious, 'Unique', true);
       
    matchedPoints = points(indexPairs(:,1), :);
    matchedPointsPrev = pointsPrevious(indexPairs(:,2), :);        
    
    % Estimate the transformation between I(n) and I(n-1).
    tforms(n) = estimateGeometricTransform(matchedPoints, matchedPointsPrev,...
        'projective', 'Confidence', 99.9, 'MaxNumTrials', 2000);
    
    % Compute T(n) * T(n-1) * ... * T(1)
    tforms(n).T = tforms(n).T * tforms(n-1).T; 
end

% Compute the output limits  for each transform
for i = 1:numel(tforms)           
    [xlim(i,:), ylim(i,:)] = outputLimits(tforms(i), [1 imageSize(i,2)], [1 imageSize(i,1)]);    
end
avgXLim = mean(xlim, 2);

[~, idx] = sort(avgXLim);

centerIdx = floor((numel(tforms)+1)/2);

centerImageIdx = idx(centerIdx);
Tinv = invert(tforms(centerImageIdx));

for i = 1:numel(tforms)    
    tforms(i).T = tforms(i).T * Tinv.T;
end

for i = 1:numel(tforms)           
    [xlim(i,:), ylim(i,:)] = outputLimits(tforms(i), [1 imageSize(i,2)], [1 imageSize(i,1)]);
end

maxImageSize = max(imageSize);

% Find the minimum and maximum output limits 
xMin = min([1; xlim(:)]);
xMax = max([maxImageSize(2); xlim(:)]);

yMin = min([1; ylim(:)]);
yMax = max([maxImageSize(1); ylim(:)]);

% Width and height of panorama.
width  = round(xMax - xMin);
height = round(yMax - yMin);

% Initialize the "empty" panorama.
panorama = zeros([height width], 'like', I);

blender = vision.AlphaBlender('Operation', 'Binary mask', ...
    'MaskSource', 'Input port');  

% Create a 2-D spatial reference object defining the size of the panorama.
xLimits = [xMin xMax];
yLimits = [yMin yMax];
panoramaView = imref2d([height width], xLimits, yLimits);

% Set for saving the camera positions
camera_x = [];
camera_y = [];
% Create the panorama.
for i = 1:numImages
    
    I = readimage(buildingScene, i);   
   
    % Transform I into the panorama.
    warpedImage = imwarp(I, tforms(i), 'OutputView', panoramaView);
                  
    % Generate a binary mask.    
    mask = imwarp(true(size(I,1),size(I,2)), tforms(i), 'OutputView', panoramaView);
    
    % Estimate the Camera Positions in panorama
    [pos_x, pos_y] = estimate_camera_pos(mask);
    camera_x = [camera_x; pos_x];
    camera_y = [camera_y; pos_y];
    
    % Overlay the warpedImage onto the panorama.
    panorama = step(blender, panorama, warpedImage, mask);
end

figure;
% Because the image was rotated for calibration brfore,
% here to plot in the nomal pointview
J = imrotate(panorama, 90, 'bilinear');
imshow(J);
hold on;
% for showing estimated camrera pos in rotated panorama
scatter(camera_x, camera_y, 180, 'filled', 'r');
%imshow(panorama);

