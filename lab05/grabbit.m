camworld_H_armworld = [[0 -1 0 310];[1 0 0 380];[0 0 1 327];[0 0 0 1]];

while true   
    clearvars -except camworld_H_armworld
    
    % Run image detection to try to find cylinders
    disp('Checking for objects...')
    [camx,camy,thetas] = imagedetection();
    
    if isempty(camx)
        pause(5)
        continue
    end
    
    % Begin converting the cylinders from camera coords to arm coords
    robx = zeros(length(camx),1);
    roby = zeros(length(robx),1);
    robz = zeros(length(robx),1);

    for i = 1 : length(robx)
        pos = eye(4);
        pos(1,4) = camx(i);
        pos(2,4) = camy(i);
        pos(3,4) = 141;

        temp = camworld_H_armworld \ pos;
        robx(i) = temp(1,4);
        roby(i) = temp(2,4);
        robz(i) = temp(3,4);

    end

    % Print object positions for early testing
    robx;
    roby;
    thetas;

    % Bounds checking to avoid collision with camera or monitors
    xthresh = 0;
    ythresh = 0;

    if robx(1) > xthresh || roby(1) < ythresh
       disp('Attempted to move OOB. Exiting...')
       return 
    end

    % Proceed with Movement
    % O= ?, A=90 , T=0
    disp('Moving to Ready Position')
    system('Puma_Ready');
    % pause(2)
%     disp('Moving to Object X/Y')
%     system(Puma_MOVEXYZ robx(1) roby(1) 0 theta+rot_offset 90 0)
%     pause(2)
%     disp('Grabbing object')
%     system(Pume_MOVEXYZ robx(1) roby(1) -186 theta+rot_offset 90 0)
%     pause(2)
%     disp('Moving to Ready Position')
%     system('Puma_Ready')

    pause(5)
    close all
end