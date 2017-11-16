clear;
clc;

im = imread('20.jpg');
figure;
imshow(im);

im_big = imresize(im, 2);
figure;
imshow(im_big);

im_sharp = imsharpen(im_big, 'Radius',2,'Amount',4);
figure;
imshow(im_sharp);

imwrite(im_sharp, '20_sharp.jpg')