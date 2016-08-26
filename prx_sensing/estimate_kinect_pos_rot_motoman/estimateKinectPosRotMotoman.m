function estimateKinectPosRotMotoman(kinectName, eval)
    
    % No parameters
    if nargin < 1
        kinectName = 'all';
        eval = 0;
    elseif nargin < 2 % One parameters
        eval = 0;
    end
    
    % Opening file
    scriptName = mfilename('fullpath');
    [currentPath, ~, ~]= fileparts(scriptName);
    fid = fopen(sprintf('%s/%s',currentPath, 'cameras_pos_rot.data'), 'w');
    
    % Making list for all cameras
    if strcmp(kinectName, 'all') == 1
        camList = {'Right'; 'Head'; 'Left'};
    else % Making list only for specified camera
        camList = {kinectName};
    end
       
    % Running position/orientation calculation funtion for specified camera in list
    for i = 1 : size(camList, 1)
        [camPos, camRot, avgErr, maxErr] = calcPosRot(camList(i), eval);

        fprintf('<%s Kinect>\n', camList{i});
        display('Camera Position(mm):');
        display(camPos);
        
        display('Camera Position(cm):');
        display(camPos/10);
        
        display('Camera Position(m):');
        display(camPos/1000);
        
        display('Rotation Matrix:');
        display(camRot);
        
        display('Axis/Angle');
        display(vrrotmat2vec(camRot));
        
        display('Quaternion[q0,q1,q2,w]');
        display(rot2quat(camRot));
        
        if eval == 1
            display(avgErr);
            display(maxErr);
        end
        
        fprintf(fid, '<%s_Kinect>\n', camList{i});
        fprintf(fid, '\tPosition(mm):\n\t\t');
        fprintf(fid, '%f ', camPos(1,:));
        fprintf(fid, '\n\tPosition(cm):\n\t\t');
        fprintf(fid, '%f ', camPos(1,:)/10);
        fprintf(fid, '\n\tPosition(m):\n\t\t');
        fprintf(fid, '%f ', camPos(1,:)/1000);
        fprintf(fid, '\n\tOrientation:\n');
        for j = 1 : size(camRot,1)
            fprintf(fid, '\t\t');
            fprintf(fid, '%f ', camRot(j,:));
            fprintf(fid, '\n');
        end
        fprintf(fid, '\tAxis/Angle:\n\t\t');
        fprintf(fid, '%f ', vrrotmat2vec(camRot));
        fprintf(fid, '\n\tQuaternion[q0,q1,q2,w]:\n\t\t');
        fprintf(fid, '%f ', rot2quat(camRot));
        fprintf(fid, '\n<\\%s_Kinect>\n', camList{i});
        fprintf(fid, '\n');
        
    end
      
    fclose(fid);
    
end