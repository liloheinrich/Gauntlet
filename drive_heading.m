clc; close all; clear all;

% 1. scan
% 2. fit
% 3. path
% 5. DRIVE!
drive('output');

function drive(datafile)
    load pathpoints
    load shapedata
    
    collectDataset_sim(datafile) % press space to activate data collection!
    pub = rospublisher('/raw_vel'); msg = rosmessage(pub);
    msg.Data = [0, 0]; send(pub, msg); % stop the robot
    
    count = 1;
    dx = xn(count+1)-xn(count);
    dy = yn(count+1)-yn(count);
    heading = atan2(dy, dx);
    placeNeato(0,0,dx,dy);

    pause(10); % wait a bit for robot to fall onto the bridge
    start = rostime('now'); % get the simulation time as a rostime obj
    start = double(start.Sec)+double(start.Nsec)*10^-9; % convert to double
    
    pause_time = .1;
    p = 1.5;
    v = .535;
    'starting loop'
    
    % control loop feeding velocity data to the neato over time
    while count < size(xn, 2)
        currTime = rostime('now');
        currTime = double(currTime.Sec)+double(currTime.Nsec)*10^-9;
        elapsedTime = currTime - start;
        
        if elapsedTime > pause_time*count
            count
            dx = xn(count+1)-xn(count);
            dy = yn(count+1)-yn(count);
            new_heading = atan2(dy, dx);
            msg.Data = [v+(heading-new_heading)*p v-(heading-new_heading)*p];
%             msg.Data = [1 1];
            send(pub, msg);
            count = count + 1;
            heading = new_heading;
        end
        pause(pause_time/10) % allows the subscriber thread to run
    end
    'done with loop'
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