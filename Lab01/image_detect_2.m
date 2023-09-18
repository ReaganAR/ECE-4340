I_left = imread('left3.ppm');

BWL=rgb2gray(I_left);

rows=2;
columns=2;

%BIL= imfill(~imbinarize(BWL),'holes');

EDGEL = edge(imfill(~imbinarize(BWL),'holes'), 'canny');

EDGELL = imdilate(EDGEL,strel('square', 3));
%EDGER = edge(BIR, 'canny');


%statsleft=regionprops(BIL, 'centroid', 'orientation', 'area');
%centroidsleft = cat(1,statsleft.Centroid);

%statsright=regionprops(BIR, 'centroid', 'orientation',  'area');
%centroidsright = cat(1,statsright.Centroid);






figure(1)
subplot(rows,columns,1)
imshow(I_left)
title('Left original image')

subplot(rows,columns,2)
imshow(BWL)
title('Left BW Image')


subplot(rows,columns,3)
imshow(EDGEL)
title('Left canny image')

subplot(rows,columns,4)
imshow(EDGELL)
title('Dilated canny')
%{
subplot(rows,columns,5)
imshow(EDGEL)
title('Left canny image')

subplot(rows,columns,6)
imshow(EDGER)
title('Right canny image')

[H,T,R] = hough(EDGEL,'RhoResolution',0.5,'Theta',-90:0.5:89);
[HR, thetaR, rhoR] = hough(EDGER);



subplot(rows,columns,7);
imshow(H,'XData',T,'YData',R,...
      'InitialMagnification','fit');
xlabel('\theta'), ylabel('\rho');
axis on, axis normal, hold on;
colormap(gca,hot);
hold off;





subplot(rows,columns,9)
hold on
imshow(BIL)
title('Left binary image')
plot(centroidsleft(:,1),centroidsleft(:,2),'b*')
hold off

subplot(rows,columns,10)
hold on
imshow(BIR)
title('Right binary image')
plot(centroidsright(:,1),centroidsright(:,2),'b*')
hold off



% 3D reconstruction
leftcal = load('/nfshome/car8kh/Downloads/VIGIR/images/Calib_Results_left.mat');
rightcal = load('/nfshome/car8kh/Downloads/VIGIR/images/Calib_Results_right.mat');

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
%}

