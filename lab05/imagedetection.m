function[x, y, z, theta] = imagedetection()

    
    I_left = imread('left30.ppm');
    I_right = imread('right30.ppm');

    BWL=rgb2gray(I_left);
    BWR=rgb2gray(I_right);

    rows=3;
    columns=2;

    BIL= imfill(~imbinarize(BWL),'holes');
    BIR= imfill(~imbinarize(BWR),'holes');

    statsleft=regionprops(BIL, 'centroid', 'orientation', 'area', 'MajorAxisLength');
    if size(statsleft, 1) > 0
        removed = 0;
        for i = 1 : size(statsleft, 1)
            i = i - removed;
            if statsleft(i).MajorAxisLength < 100 || statsleft(i).Area > 8000 || statsleft(i).Area < 4000
                statsleft(i) = [];
                removed = removed + 1;
            end
        end
    end
    
    
    centroidsleft = cat(1,statsleft.Centroid);
    orientsleft = cat(1,statsleft.Orientation);

    statsright=regionprops(BIR, 'centroid', 'orientation',  'area', 'MajorAxisLength');
    
    if size(statsright, 1) > 0
        removed = 0;
        for i = 1 : size(statsright, 1)
            i = i - removed;
            if statsright(i).MajorAxisLength < 100 || statsright(i).Area > 8000 || statsright(i).Area < 4000
                statsright(i) = [];
                removed = removed + 1;
            end
        end
    end
    
    centroidsright = cat(1,statsright.Centroid);
    orientsright = cat(1,statsright.Orientation);

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
    if length(centroidsleft) > 0
        plot(centroidsleft(:,1),centroidsleft(:,2),'b*')
        for i = 1 : length(orientsleft)
            hlen = statsleft(i).MajorAxisLength/2;
            cosOrient = cosd(orientsleft(i));
            sinOrient = sind(orientsleft(i));
            xcoords = centroidsleft(i,1) + hlen * [-cosOrient cosOrient];
            ycoords = centroidsleft(i,2) + hlen * [sinOrient -sinOrient];
            line(xcoords, ycoords);
        end
    end
    hold off

    subplot(rows,columns,6)
    hold on
    imshow(BIR)
    title('Right binary image')
    if length(centroidsright) > 0
        plot(centroidsright(:,1),centroidsright(:,2),'b*')
        for i = 1 : length(orientsright)
            hlen = statsright(i).MajorAxisLength/2;
            cosOrient = cosd(orientsright(i));
            sinOrient = sind(orientsright(i));
            xcoords = centroidsright(i,1) + hlen * [-cosOrient cosOrient];
            ycoords = centroidsright(i,2) + hlen * [sinOrient -sinOrient];
            line(xcoords, ycoords);
        end
    end
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
    
    x = [];
    y = [];
    theta = [];
    
    if length(centroidsleft) > 0
        numObjs = length(centroidsleft(:,1));
        M =zeros(numObjs, 3);
        for i=1:numObjs
            uvleft = centroidsleft(i,:);
            uvright = centroidsright(i,:);
            M(i,:) = findXYZ(uvleft, uvright, Pleft, Pright);
        end
    
    x = M(:,1);
    y = M(:,2);
    z = M(:,3);
    theta = orientsleft;
    end
end
