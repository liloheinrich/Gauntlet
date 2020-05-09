clc; close all; clear all;

% 1. scan
% 2. fit
% 3. path
% 4. velocities
% 5. DRIVE!
drive('velocities', 'output');

function drive(pathfile, datafile)
    collectDataset_sim(datafile) % press space to activate data collection!
    pub = rospublisher('/raw_vel'); msg = rosmessage(pub);
    msg.Data = [0, 0]; send(pub, msg); % stop the robot

    placeNeato(0, 0, 0, 0);

    pause(5); % wait a bit for robot to fall onto the bridge
    start = rostime('now'); % get the simulation time as a rostime obj
    start = double(start.Sec)+double(start.Nsec)*10^-9; % convert to double
    pause_time = .1
    count = 1;
    
    V = .5;
    p_ang = .5;
    
    global pose twist;
    pose = 0;
    twist = 0;
    
    sub_odom = rossubscriber('/odom', @process);
    function process(sub, msg)
        pose = msg.Pose.Pose
        twist = msg.Twist.Twist
        
        twist.Linear
        twist.Angluar
        pose.Point
        pose.Quanternion
    end
    
    scan('scandata')
    fit('scandata', 'shapedata')
    lambda = .001; sigma = .975; xi = 0; yi = 0; max_pts = 100;
    [xn, yn] = calculate_all('shapedata', xi, yi, lambda, sigma, max_pts);
    save('pathpoints', 'xn', 'yn')
    
    % control loop feeding velocity data to the neato over time
    dist_from_center = sqrt((pose.x-center(1))^2+(pose.y-center(2))^2);
    while dist_from_center > radius
%         proportional controllers?
        currTime = rostime('now');
        currTime = double(currTime.Sec)+double(currTime.Nsec)*10^-9;
        elapsedTime = currTime - start;
        
        if elapsedTime > pause_time*count
            diff = twist - atan(yn(count)/xn(count))
            msg.Data = [V-p_ang*diff,V+p_ang*diff];
            send(pub, msg);
            count = count + 1;
        end
        
        dist_from_center = sqrt((pose.x-center(1))^2+(pose.y-center(2))^2)
        pause(pause_time) % allows the subscriber thread to run
    end
    disp(['Elapsed time: ', num2str(elapsedTime)]);
    msg.Data = [0, 0]; send(pub, msg); 
end

function [xn yn] = calculate_all(input, xi, yi, lambda, sigma, max_pts)
    load(input);
    syms X Y Z;
    inc = .1;
    for i=1:size(endpoints, 1)
        for j = min(endpoints(i,:,1)):inc:max(endpoints(i,:,1))
            for k = min(endpoints(i,:,2)):inc:max(endpoints(i,:,2))
                Z = Z + log(sqrt((X-j).^2+(Y-k).^2));
            end
        end
    end
    for theta = 0:inc:2*pi
        a = center(1)+radius.*cos(theta);
        b = center(2)+radius.*sin(theta);
        Z = Z - log(sqrt((X-a).^2+(Y-b).^2))*2;
    end
    G = gradient(Z, [X, Y]);
    dist_from_center = sqrt((xi-center(1))^2+(yi-center(2))^2);
    xn = zeros(1,1);
    yn = zeros(1,1);
    xn(1) = xi;
    yn(1) = yi;
    i = 2;
    while dist_from_center > radius & i < max_pts
        fx = subs(G(1), [X Y], [xn(i-1) yn(i-1)]);
        fy = subs(G(2), [X Y], [xn(i-1) yn(i-1)]);
        xn(i) = lambda*fx + xn(i-1);
        yn(i) = lambda*fy + yn(i-1);
        hold on
        plot(xn(i), yn(i), 'gx')
%         pause(.25)
        hold off
        dist_from_center = sqrt((xn(i)-center(1))^2+(yn(i)-center(2))^2);
        lambda = lambda*sigma;
        i = i + 1;
    end
    graph(radius, center, endpoints)
    legend('Path points')
    title('Path to BoB using gradient descent')
end

% graph gauntlet shapedata that was calculated using ransac_fit.m
function graph(radius, center, endpoints)
    hold on
    circlepts = zeros(2,360);
    for angle=1:360
        circlepts(:,angle) = [radius*cosd(angle)+center(1), radius*sind(angle)+center(2)];
    end
    plot(circlepts(1,:), circlepts(2,:), 'm')
    plot(center(:,1), center(:,2), 'mx')
    plot(0,0,'bx')
    for i=1:size(endpoints, 1)
        plot(endpoints(i,:,1), endpoints(i,:,2), 'r')
    end
    axis equal;
    xlim([-2, 3]);
    ylim([-3, 1]);
    xlabel('[m]')
    ylabel('[m]')
    hold off;
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

function scan(datasetname)
% Method for collecting lidar scan data from Neato. 
% - To launch, call this method. 
% - To start execution of the program, press space bar. 
% - To stop execution of the program, close the figure window. 
% - Collected data (ranges, ange_increment) is stored under 'datasetname.mat'

    function myCloseRequest(src,callbackdata)
        % Close request function to display a question dialog box
        % get rid of subscriptions to avoid race conditions
        clear ranges;
        clear angle_increment;
        delete(gcf)
    end

    function processScan(sub, msg)
        ranges = msg.Ranges;
        angle_increment = msg.AngleIncrement;
    end

    function keyPressedFunction(fig_obj, eventDat)
        % Convert a key pressed event into a twist message and publish it
        ck = get(fig_obj, 'CurrentKey');
        switch ck
            case 'space'
                if collectingData
                    collectingData = false;
                    save(datasetname, 'ranges', 'angle_increment');
                    disp('Stopping dataset collection');
                else
                    collectingData = true;
                    disp('Starting dataset collection');
                end
        end
    end

    global ranges angle_increment;
    collectingData = false;
    ranges = zeros(1,1);
    angle_increment = 0;
    sub_scan = rossubscriber('/scan', @processScan);

	f = figure('CloseRequestFcn',@myCloseRequest);
    title('Dataset Collection Window');
    set(f,'WindowKeyPressFcn', @keyPressedFunction);
end

function fit(input, output)
    % load scan data that was collected using collectScan.m
    load(input)

    % create theta based on angle_increment for each data point in ranges
    theta = zeros(size(ranges, 1), 1);
    for i=1:size(ranges, 1)
        theta(i) = i*angle_increment;
    end

    % clean data, remove invalid data that exceeds lidar range/room size
    index=find(ranges~=0 & ranges<3);
    ranges=ranges(index);
    theta=theta(index);

    % convert to cartesian x, y
    [x,y]=pol2cart(theta,ranges);
    cart=[x y];

    % graph the data points
    hold on; plot(cart(:,1), cart(:,2),'k.'); hold off;

    % setup variables and run targeted circle fit first
    d = .005; n = 1000;
    tol = .1; r = .25;

    % circle fit looking for BoB in certain area on map
    % [outliers calc_r center] = circlefit(cart, d, n, r, tol);

    % circle fit using discriminating criteria (ex. point dist, radius,
    % near inliers) to identify the best circle
    [outliers radius center] = circlefit2(cart, d, n, r, tol);

    stop = 2; k = .5;
    [outliers endpoints] = linefit_multiple(outliers, d, n, k, stop);

    % save line and circle data
    save(output, 'endpoints', 'radius', 'center');
end

function [outliers endpoints] = linefit_multiple(cart, d, n, k, cutoff)
    outliers = cart;
    endpoints = [];
    while size(outliers, 1) > cutoff
        [endline inlier] = linefit(outliers, d, n, k);
        outliers(inlier,:) = [];
        endpoints(size(endpoints, 1)+1,:,:) = (endline);
        pause(.25)
    end
end

function graph_line(endline)
    hold on; 
    plot(endline(:, 1), endline(:, 2), 'bo')
    plot(endline(:,1), endline(:,2), 'g')
    
    legend('Data points','Fit lines','Line endpoints')
    axis equal;
    xlim([-2, 3]);
    ylim([-3, 1]);
    xlabel('[m]')
    ylabel('[m]')
    title('Initial position scan data with line fits')
    hold off;
end

function graph_circle(center, r)
    hold on
    circlepts = zeros(2,360);
    for angle=1:360
        circlepts(:,angle) = [r*cosd(angle)+center(1), r*sind(angle)+center(2)];
    end
    plot(circlepts(1,:), circlepts(2,:), 'g')
    
    legend('Data points','Fit lines')
    axis equal;
    xlim([-2, 3]);
    ylim([-3, 1]);
    xlabel('[m]')
    ylabel('[m]')
    title(' ')
    hold off;
end

function [endline inliers] = linefit(cart, d, n, k)
    inliers = []; % indeces in cart of inlying points
    endline = [0 0; 0 0]; % endpoints of line on line
    m = 0; % slope of best line
    b = 0; % intercept of best line
    
    for i=1:n
        rand_indx = randi([1,size(cart,1)],1,2); % random indeces pair
        new_endpoints = [cart(rand_indx(1),1) cart(rand_indx(1),2); cart(rand_indx(2),1) cart(rand_indx(2),2)];
        [new_inliers new_endline new_m new_b] = calc_inliers(cart, d, new_endpoints);
        
        largest_gap = calc_largest_gap(cart(new_inliers,:), new_endpoints(1,:));
        if largest_gap > k
            new_inliers = []; % num of inlying points
        end
        
        if (size(new_inliers,1) > size(inliers,1))
            inliers = new_inliers;
            endline = new_endline;
            m = new_m;
            b = new_b;
        end
    end
    graph_line(endline);
end

function [min_dist, index] = closest_point_dist(points, point)
    min_dist = 999999999;
    index = 0;
    for i=1:size(points, 1)
        if points(i,1) == point(1) & points(i,2) == point(2)
            min_dist = 0;
            index = i;
        end
        if points(i,1) ~= point(1) | points(i,2) ~= point(2)
            dist = sqrt((points(i,1)-point(1))^2 + (points(i,2)-point(2))^2);
            if dist < min_dist & dist ~= 0
                min_dist = dist;
                index = i;
            end
        end
    end
end

function max_gap = calc_largest_gap(points, point)
    points_left = points;
    max_gap = 0;
    
    [min_dist, index] = closest_point_dist(points_left, point);
    if min_dist > max_gap
        max_gap = min_dist;
    end
    
    while size(points_left,1) > 1
        point = points_left(index,:);
        points_left(index,:) = [];
        [min_dist, index] = closest_point_dist(points_left, point);
        if min_dist > max_gap
            max_gap = min_dist;
        end
    end
end

function [inliers endline m b] = calc_inliers(cart, d, endpoints)
    m = (endpoints(1,2)-endpoints(2,2))/(endpoints(1,1)-endpoints(2,1)); % rise/run
    b = endpoints(1,2)-m*endpoints(1,1); % b = y - mx
    dist = sqrt((endpoints(1,1)-endpoints(2,1))^2 + (endpoints(1,2)-endpoints(2,2))^2);
    
    bn_endpoints = endpoints(:,2) + endpoints(:,1)/m; % solve b = y + x/m
    x_endpoints = (bn_endpoints - b)/(m + 1/m); % solve mx+b = -x/m+bn for x
    y_endpoints = m*x_endpoints + b;
    endline = [x_endpoints y_endpoints]; % intersection on line tangent to endpoints
    
    inliers = [];
    for i=1:size(cart, 1)
        bn = cart(i, 2) + cart(i, 1)/m; % solve b = y + x/m
        x = (bn - b)/(m + 1/m); % solve mx+b = -x/m+bn for x
        y = m*x + b;
        dist_tan = sqrt((cart(i,1)-x)^2 + (cart(i,2)-y)^2);% sqrt(dx^2 + dy^2)
        
        if dist_tan <= d
            inliers = [inliers; i];
            
            if size(inliers, 1) == 1
                endpoints(1,:) = cart(i,:);
                endline(1,:) = [x y];
            end
            if size(inliers, 1) == 2
                if endline(1,1) < x
                    endpoints(2,:) = cart(i,:);
                    endline(2,:) = [x y];
                end
                if endline(1,1) > x
                    endpoints(2,:) = endpoints(1,:);
                    endline(2,:) = endline(1,:);
                    endpoints(1,:) = cart(i,:);
                    endline(1,:) = [x y];
                end
            end
            if size(inliers, 1) > 2
                if x > endline(2,1)
                    endpoints(2,:) = cart(i,:);
                    endline(2,:) = [x y];
                end
                if x < endline(1,1)
                    endpoints(1,:) = cart(i,:);
                    endline(1,:) = [x y];
                end
            end
        end
    end
end

function [outliers, calc_r, center] = circlefit(cart, d, n, r, tol)
    calc_r = r;
    inliers = zeros(0,2);
    center = zeros(0,2);
    for k=1:n
        % Looking in a particular area on the graph instead of guessing
        % blindly. signifiantly improves recognition at the expense of
        % having to know map layout.
        xs = cart(:,1);
        ys = cart(:,2);
        indexx=find(xs > .2 & xs < .8);
        indexy=find(ys > -2.8 & ys < -2.2);
        index_circ=intersect(indexx,indexy);
        circx=xs(index_circ);
        circy=ys(index_circ);
        A = [circx circy ones(size(circx))];
        b = -circx.^2 - circy.^2;

        % linear regression, solving for center and radius
        w = A\b;
        new_center = [-w(1)/2 -w(2)/2];
        new_r = sqrt(new_center(1).^2 + new_center(2).^2 - w(3));
        
        % counting up the inliers
        new_inliers = [];
        for i=1:size(cart, 1)
            dist = abs(sqrt((cart(i,1)-new_center(1)).^2 + (cart(i,2)-new_center(2)).^2) - new_r);
            if dist < d
                new_inliers = [new_inliers; i];
            end
        end
    
        % seeing if it's a better match than the previous one
        if abs(new_r - r) < tol  && size(new_inliers,1) > size(inliers,1)
            inliers = new_inliers;
            calc_r = new_r;
            center = new_center;
        end
    end
    
    graph_circle(center, calc_r);
    outliers = cart;
    outliers(inliers,:) = [];
end

function [outliers, calc_r, center] = circlefit2(cart, d, n, r, tol)
    calc_r = r;
    inliers = zeros(0,2);
    center = zeros(0,2);
    for k=1:n
        d1 = 2*r+1;
        d2 = 2*r+1;
        while d1 > 2*r || d2 > 2*r
            % Pick 3 random points and try to draw a circle through them
            candidates = datasample(cart, 3, 'Replace', false);
            d1 = sqrt((candidates(1,1)-candidates(2,1)).^2 + (candidates(1,2)-candidates(2,2)).^2);
            d2 = sqrt((candidates(2,1)-candidates(3,1)).^2 + (candidates(2,2)-candidates(3,2)).^2);
        end
        % linear regression, solving for center and radius
        A = [candidates(:,1) candidates(:,2) ones(size(candidates, 1),1)];
        b = -candidates(:,1).^2 - candidates(:,2).^2;
        w = A\b;
        b2=A*w;
        new_center = [-w(1)/2 -w(2)/2];
        new_r = sqrt(new_center(1).^2 + new_center(2).^2 - w(3));
        
        if abs(new_r - r) < tol
            % counting up the inliers and near inliers
            near_inliers = [];
            new_inliers = [];
            for i=1:size(cart, 1)
                dist = abs(sqrt((cart(i,1)-new_center(1)).^2 + (cart(i,2)-new_center(2)).^2) - new_r);
                if dist < d
                    new_inliers = [new_inliers; i];
                end
                if dist < 2*d && dist > d
                    near_inliers = [near_inliers; i];
                end
            end
            % seeing if it's a better match than the previous one
            if size(new_inliers,1) > size(inliers,1) && size(near_inliers, 1) == 0
                inliers = new_inliers;
                calc_r = new_r;
                center = new_center;
            end
        end
    end
    graph_circle(center, calc_r);
    outliers = cart;
    outliers(inliers,:) = [];
end