clc; close all; clear all;

% 1. scan
% 2. fit
% 3. path
% 4. velocities
% 5. DRIVE!
drive('velocities', 'output');

function drive(pathfile, datafile)
    % Makes neato drive path given by pathdata, R, and That in 'pathfile',
    % collects and records the encoder/accelerometer data to 'datafile'.
    t = []; syms t;
    S = load(pathfile);
    vel = S.pathdata(:,4:5);
    time = S.pathdata(:,1);

    disp(['Expected time: ', num2str(time(size(time,1)))]);
    collectDataset_sim(datafile) % press space to activate data collection!
    pub = rospublisher('/raw_vel'); msg = rosmessage(pub);
    msg.Data = [0, 0]; send(pub, msg); % stop the robot

    bridgeStart = double(subs(S.R,t,0));
    startingThat = double(subs(S.That,t,0));
    placeNeato(bridgeStart(1),  bridgeStart(2), startingThat(1), startingThat(2));

    pause(5); % wait a bit for robot to fall onto the bridge
    start = rostime('now'); % get the simulation time as a rostime obj
    start = double(start.Sec)+double(start.Nsec)*10^-9; % convert to double
    pause_time = time(size(time,1))/size(vel,1); % time between each datapoint
    count = 1;
    
    % control loop feeding velocity data to the neato over time
    while count <= size(vel,1)
        currTime = rostime('now');
        currTime = double(currTime.Sec)+double(currTime.Nsec)*10^-9;
        elapsedTime = currTime - start;
        
        if elapsedTime > pause_time*count
%             vel(count,:)
            msg.Data = [vel(count,2) vel(count,1)];
            send(pub, msg);
            count = count + 1;
        end
        pause(pause_time/10) % allows the subscriber thread to run
    end
    disp(['Elapsed time: ', num2str(elapsedTime)]);
    msg.Data = [0, 0]; send(pub, msg); 
end

function placeNeato(posX, posY, headingX, headingY)
% Places the neato at the given x, y position and heading.

    svc = rossvcclient('gazebo/set_model_state');
    ms = rosmessage(svc);

    ms.ModelState.ModelName = 'neato_standalone';
    startYaw = atan2(headingY, headingX);
    quat = eul2quat([startYaw 0 0]);

    ms.ModelState.Pose.Position.X = posX;
    ms.ModelState.Pose.Position.Y = posY;
    ms.ModelState.Pose.Position.Z = 1.0;
    ms.ModelState.Pose.Orientation.W = quat(1);
    ms.ModelState.Pose.Orientation.X = quat(2);
    ms.ModelState.Pose.Orientation.Y = quat(3);
    ms.ModelState.Pose.Orientation.Z = quat(4);

    % put the robot in the appropriate place
    ret = call(svc, ms);
end

function collectDataset_sim(datasetname)
% This script provides a method for collecting a dataset from the Neato
% sensors suitable for plotting out a 3d trajectory.
% The collected data is stored in a variable called dataset with row format:
% [timestamp, positionLeft, positionRight, AccelX, AccelY, AccelZ];
    function myCloseRequest(src,callbackdata)
        % Close request function 
        % to display a question dialog box
        % get rid of subscriptions to avoid race conditions
        clear sub_encoders;
        clear sub_accel;
        delete(gcf)
    end

    function processAccel(sub, msg)
        % Process the encoders values by storing by storing them into
        % the matrix of data.
        lastAccel = msg.Data;
    end

    function processEncoders(sub, msg)
        % Process the encoders values by storing by storing them into
        % the matrix of data.
        if ~collectingData
            return;
        end
        currTime = rostime('now');
        currTime = double(currTime.Sec)+double(currTime.Nsec)*10^-9;
        elapsedTime = currTime - start;
        dataset(encoderCount + 1,:) = [elapsedTime msg.Data' lastAccel'];
        encoderCount = encoderCount + 1;
    end

    function keyPressedFunction(fig_obj, eventDat)
        % Convert a key pressed event into a twist message and publish it
        ck = get(fig_obj, 'CurrentKey');
        switch ck
            case 'space'
                if collectingData
                    collectingData = false;
                    dataset = dataset(1:encoderCount, :);
                    save(datasetname, 'dataset');
                    disp('Stopping dataset collection');
                else
                    start = rostime('now');
                    start = double(start.Sec)+double(start.Nsec)*10^-9;
                    encoderCount = 0;
                    dataset = zeros(10000, 6);
                    collectingData = true;
                    disp('Starting dataset collection');
                end
        end
    end

    global dataset start encoderCount lastAccel;
    lastAccel = [0; 0; 1];      % set this to avoid a very unlikely to occur race condition
    collectingData = false;
    sub_encoders = rossubscriber('/encoders', @processEncoders);
    sub_accel = rossubscriber('/accel', @processAccel);

	f = figure('CloseRequestFcn',@myCloseRequest);
    title('Dataset Collection Window');
    set(f,'WindowKeyPressFcn', @keyPressedFunction);
end
