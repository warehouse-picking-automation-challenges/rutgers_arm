%{
    @brief This function estimates the position and orientation of kinects on Motoman in world frame.
           camPos estimated position
           camRot estimated orientation
           avgErr average error in case of evaluation
           maxErr maximum error in case of evaluation
    
    @param kinectName name of the desired kinect on Motoman('Right', 'Left', 'Head')
    @param eval       boolean variable for turnning on(1) and turninng off(0) the evaluation process
%}

function [camPos, camRot, avgErr, maxErr] = calcPosRot(kinectName, eval)

    if strcmp(kinectName, 'Right') == 1 % Right Kinect
        
        % Checkerboard start position(top left) in world frame - Unit 'mm'
        startX = 41.1 * 10;
        startY = 13.5 * 10;
        
        % Reading reference image
        imOrig = imread('right_new_ch_1.png');

        % Setting camera matrix
        cameraParams = cameraParameters('IntrinsicMatrix', [1.0315606674018809e+03, 0, 0; 0, 1.0396643186093827e+03, 0; 9.3819707824606667e+02, 5.2479200092365159e+02, 1.0], ...
                                        'RadialDistortion', [4.2979576575695108e-02, -7.5057827653960810e-02, 2.4177999279291429e-02], ...
                                        'TangentialDistortion', [-2.1316113787823504e-03, -4.0519543740441090e-03]);
                                    
        % Correct kinect depth value - Unit 'mm'
        kinectZ = 0;
        
        % Length of each cell in checkerboard - Unit 'mm'
        cellLength = 23;
    
        % Number of cells in checkerboard in each axis in world frame
        Y = 9;
        X = 6;
    
        % Generating world coordinate of corners in checkerboard in world frame - Unit 'mm'
        worldPoints = zeros(X*Y, 2);

        for y = 1 : Y
            tmp = startY - (y-1) * cellLength;
            for x = 1 : X
                worldPoints((y-1)*6+x, 1) = startX - (x-1) * cellLength; 
                worldPoints((y-1)*6+x, 2) = tmp; 
            end
        end

    elseif strcmp(kinectName, 'Left') == 1 % Left Kinect
    
        % Checkerboard start position(top left) in world frame - Unit 'mm'
        startX = 41.1 * 10;
        startY = 13.5 * 10;
        
        % Reading reference image
        imOrig = imread('left_new_ch_1.png');

        % Setting camera matrix
        cameraParams = cameraParameters('IntrinsicMatrix', [1.0347053803690899e+03, 0.0, 0.0; 0.0, 1.0413509558351850e+03, 0.0; 9.3814634484811677e+02, 5.4018554070494338e+02, 1.0], ...
                                        'RadialDistortion',[3.5519610137614226e-02, -6.3421865464156221e-02,1.7050136505265708e-02], ...
                                        'TangentialDistortion',[-3.5786562723541080e-03, -4.8376259436128163e-03]);
                      
        % Correct kinect depth value - Unit 'mm'
        kinectZ = 0;
        
        % Length of each cell in checkerboard - Unit 'mm'
        cellLength = 23;
    
        % Number of cells in checkerboard in each axis in world frame
        Y = 9;
        X = 6;
    
        % Generating world coordinate of corners in checkerboard in world frame - Unit 'mm'
        worldPoints = zeros(X*Y, 2);

        for y = 1 : Y
            tmp = startY - (y-1) * cellLength;
            for x = 1 : X
                worldPoints((y-1)*6+x, 1) = startX - (x-1) * cellLength; 
                worldPoints((y-1)*6+x, 2) = tmp; 
            end
        end
                                        
    elseif strcmp(kinectName, 'Head') == 1 % Head Kinect
    
        % Checkerboard start position(top left) in world frame - Unit 'mm'
        startX = 148.3 * 10; %150.5 * 10;
        startY = 6.7  * 10; %8.8 * 10;
        
        % Reading reference image
        imOrig = imread('head_new_ch_1.png');

        % Setting camera matrix
        cameraParams = cameraParameters('IntrinsicMatrix', [1.0354732123361073e+03, 0, 0; 0, 1.0438219307398404e+03, 0; 9.3825584330626793e+02, 5.2725899819786900e+02, 1.0], ...
                                        'RadialDistortion', [2.8061465000397214e-02, -5.0689223106997854e-02, 1.3439799779382946e-02], ...
                                        'TangentialDistortion', [-1.4695684261733179e-03, -2.1678013200548524e-03]);
                                    
        % Correct kinect depth value - Unit 'mm'
        kinectZ = 0;
        
        % Length of each cell in checkerboard - Unit 'mm'
        cellLength = 23;
    
        % Number of cells in checkerboard in each axis in world frame
        Y = 9;
        X = 6;
    
        % Generating world coordinate of corners in checkerboard in world frame - Unit 'mm'
        worldPoints = zeros(X*Y, 2);

        for y = 1 : Y
            tmp = startY - (y-1) * cellLength;
            for x = 1 : X
                worldPoints((y-1)*6+x, 1) = startX - (x-1) * cellLength; 
                worldPoints((y-1)*6+x, 2) = tmp; 
            end
        end
 
    end

    % Undistorting the image
    im = undistortImage(imOrig, cameraParams);

    % Finding image coordinate of corners in image frame
    [imagePoints, ~] = detectCheckerboardPoints(im);

    % Estimating the extrinsic parameters of camera
    [rotationMatrix, translationVector] = extrinsics(imagePoints, worldPoints, cameraParams);

    % Calculating camera position and rotation
    camPos = -translationVector * transpose(rotationMatrix);
    camRot =  rotationMatrix;
    
    if strcmp(kinectName, 'Head') == 1
        camPos = minus(camPos, [0 0 (60.7*10)]);
        
        R90 = [  0     0     -1 ;
                 0     1     0 ;
                1     0     0 ];
            
        camPos = R90 * camPos';
        camPos = camPos';
        camRot = R90 * camRot;
    end
    
    %eul = [pi/2 0 0];
    %rotmZYX = eul2rotm(eul);
    %camRot = camRot*rotmZYX;
    
    % Evaluation
    if eval == 1
        reprojectWPs = pointsToWorld(cameraParams, rotationMatrix, translationVector, imagePoints);
        diff = worldPoints - reprojectWPs;
        avgErr = [sum(diff) / size(worldPoints, 1) kinectZ-camPos(1,3)];
        maxErr = [max(diff) kinectZ-camPos(1,3)];
    else
        avgErr = [];
        maxErr = [];
    end
    
end

% imshow(im); hold on; plot(imagePoints(9:end,1), imagePoints(9:end,2), 'ro');