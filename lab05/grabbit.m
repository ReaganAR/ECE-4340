camworld_H_armworld = [[0 -1 0 310];[1 0 0 380];[0 0 1 327];[0 0 0 1]];

while true   
    clearvars -except camworld_H_armworld % Make sure no cylinders exist from previous iterations
    
    % Run image detection to try to find cylinders
    [returncode, ~] = system('save_single_image puma2 30');
    disp('Checking for objects...')
    [camx,camy,thetas] = imagedetection();
    
    % If no cylinders are present, check again in 5 seconds
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
        pos(3,4) = 141; % Hardcoded based on tool length to allow easy transformation to -186

        temp = camworld_H_armworld \ pos; % inv(CamHarm) * pos
        robx(i) = temp(1,4);
        roby(i) = temp(2,4);
        robz(i) = temp(3,4);
    end

    % Bounds checking to avoid collision with camera or monitors
    xthresh = 0;
    ythresh = 113;
    
    if robx(1) > xthresh || roby(1) < ythresh
       disp('Attempted to move OOB. Exiting...')
       return 
    end

    % Proceed with Movement
    % O= ?, A=90 , T=0
    disp('Moving to Object X/Y');  
    system(sprintf('PumaMoveXYZOAT %d %d %d %d %d %d', robx(1), roby(1), 0, 270 + thetas(1), 90, 0));
    pause(3)
    system('openGripper &');
    disp('Grabbing object')
    system(sprintf('PumaMoveXYZOAT %d %d %d %d %d %d', robx(1), roby(1), -176, 270 + thetas(1), 90, 0));
    pause(2)
    system('closeGripper');
    system(sprintf('PumaMoveXYZOAT %d %d %d %d %d %d', robx(1), roby(1), 0, 270 + thetas(1), 90, 0));
    pause(1)
    disp('Moving to Ready Position')
    system('Puma_READY');
    pause(5)
    system('openGripper &');
    pause(2)
    system('closeGripper');
    close all
end