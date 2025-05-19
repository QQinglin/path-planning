image = imread('map.png');
grayImage = im2gray(image);
filteredImage = medfilt2(grayImage);
enhancedImage = histeq(filteredImage);
