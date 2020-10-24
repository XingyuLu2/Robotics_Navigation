function [pos_x, pos_y] = estimate_camera_pos(mask)
[columns, rows] = size(mask);
disp([columns rows]);

img_pos_x = [];
img_pos_y = [];
for i = 1:columns
 for j = 1:rows 
   if mask(i,j) == 1
     img_pos_x = [img_pos_x; i];
     continue;
   end
 end
end

xMin = min(img_pos_x);
xMax = max(img_pos_x);

pos_x = int64((xMax + xMin) / 2);

for i = 1:rows
    if mask(pos_x, i) == 1
        img_pos_y = [img_pos_y; i];
    end
end

yMin = min(img_pos_y);
yMax = max(img_pos_y);

pos_y = int64((yMax + yMin) / 2);
disp([pos_x, pos_y]);
imshow(mask);figure;