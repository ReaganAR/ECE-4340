I_left = imread('left3.ppm');
I_right = imread('right3.ppm');

BWL=rgb2gray(I_left);
BWR=rgb2gray(I_right);

rows=3;
columns=2;

BIL= imfill(~imbinarize(BWL),'holes');
BIR= imfill(~imbinarize(BWR),'holes');

statsleft=regionprops(BIL, 'centroid', 'orientation', 'area');
centroidsleft = cat(1,statsleft.Centroid);

statsright=regionprops(BIR, 'centroid', 'orientation',  'area');
centroidsright = cat(1,statsright.Centroid);

figure(1)
subplot(rows,columns,1)
imshow(I_left)
title('Left original image')

subplot(rows,columns,2)
imshow(I_right)
title('Right original image')

subplot(rows,columns,3)
imshow(BWL)
title('Left BW image')

subplot(rows,columns,4)
imshow(BWR)
title('Right BW image')
line([5,10],[5,10])


subplot(rows,columns,5)
hold on
imshow(BIL)
title('Left binary image')
plot(centroidsleft(:,1),centroidsleft(:,2),'b*')
hold off

subplot(rows,columns,6)
hold on
imshow(BIR)
title('Right binary image')
plot(centroidsright(:,1),centroidsright(:,2),'b*')
hold off



% 3D reconstruction
leftcal = load('images/Calib_Results_left.mat');
rightcal = load('images/Calib_Results_right.mat');

kleft = leftcal.KK;
kright = rightcal.KK;

Rleft = leftcal.Rc_1;
Rright = rightcal.Rc_1;

Tleft = leftcal.Tc_1;
Tright = rightcal.Tc_1;

Pleft = kleft*[Rleft, Tleft];
Pright = kright*[Rright, Tright];

numObjs = length(centroidsright(:,1));
M =zeros(numObjs, 3);
for i=1:numObjs
    uvleft = centroidsleft(i,:);
    uvright = centroidsright(i,:);
    M(i,:) = findXYZ(uvleft, uvright, Pleft, Pright);
end

