% Define the size of the map
mapWidth = 1600;
mapHeight = 1600;

% Initialize the map with ones (free space)
map = ones(mapHeight, mapWidth) * 255;

% Add some obstacles (set specific regions to zero)
% Example: Add a rectangular obstacle
map(200:300, 300:400) = 0;

map(1200:1500, 1200:1500) = 0;
map(800:1200, 500:800) = 0;

% Add another obstacle
map(400:500, 100:200) = 0;

% Save the map as a PNG file
imwrite(uint8(map), 'map.png');

% Display the map
imshow(map);

