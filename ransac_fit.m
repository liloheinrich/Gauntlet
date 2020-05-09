% used to find BoB and line segments of walls/obstacles
clc; clf; close all; clear all;
fit('data_files/scandata', 'data_files/shapedata')

function fit(input, output)
    load(input) % lidar scan data (ranges, angle_increment)

    % create theta from angle_increment for each data point in ranges
    theta = zeros(size(ranges, 1), 1);
    for i=1:size(ranges, 1)
        theta(i) = i*angle_increment;
    end

    % clean data, remove invalid data that exceeds lidar range/room size
    index=find(ranges~=0 & ranges<3);
    ranges=ranges(index);
    theta=theta(index);

    [x,y]=pol2cart(theta,ranges); % convert to cartesian x, y
    x = x - .084; % lidar offset from axle center
    cart=[x y];

    % graph the data points
    hold on; plot(cart(:,1), cart(:,2),'k.'); hold off;

    % setup variables and run targeted circle fit first
    d = .0075; % maximum inlier distance
    n = 1000; % number of ransac attempts
    r = .25; % radius of circle
    tol = .1; % percent tolerance on radius

    [outliers radius center] = circlefit(cart, d, n, r, tol);

    stop = 2; % how many outliers left to stop fitting at
    k = .5; % maximum gap size
    [outliers endpoints] = linefit_multiple(outliers, d, n, k, stop);

    % save line and circle data
    save(output, 'endpoints', 'radius', 'center');
end

function [outliers endpoints] = linefit_multiple(cart, d, n, k, cutoff)
% fit lines until the cutoff number of outliers has been reached

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
% graph line segments based on the given line endpoints

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
% graph circle based on the given center and radius

    % calculating points evenly spaced along circle perimeter
    circlepts = zeros(2,360);
    for angle=1:360
        circlepts(:,angle) = [r*cosd(angle)+center(1), r*sind(angle)+center(2)];
    end
    
    hold on
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
% fits the line segment through 2 random points with the greatest number of
% inliers with n number of attempts, d maximum inlier distance, and k
% largest gap size in dataset cart

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
            new_inliers = []; % invalid solution if it has big gaps
        end
        
        if (size(new_inliers,1) > size(inliers,1)) % check if it's best yet
            inliers = new_inliers;
            endline = new_endline;
            m = new_m;
            b = new_b;
        end
    end
    graph_line(endline);
end

function [min_dist, index] = closest_point_dist(points, point)
% calculate the minimum distance from 'point' to any other point in 'points'

    min_dist = 999999999; % large number because 'Inf' wasn't working
    index = 0; % closest point index
    for i=1:size(points, 1)
        % if it's the same point, dist is 0, save index
        if points(i,1) == point(1) & points(i,2) == point(2)
            min_dist = 0;
            index = i;
        end
        
        % if it's not the same point, calculate distance and if it's
        % closer, save dist and index
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
% calculates the biggest gap from one point to another starting from 'point'

    points_left = points; % keep track of points left
    max_gap = 0; % keep track of biggest gap
    
    [min_dist, index] = closest_point_dist(points_left, point);
    if min_dist > max_gap
        max_gap = min_dist;
    end
    
    while size(points_left,1) > 1
        % go through all points, removing ones already used, getting the
        % minimum distance from that point to any other that's left
        
        point = points_left(index,:);
        points_left(index,:) = [];
        [min_dist, index] = closest_point_dist(points_left, point);
        if min_dist > max_gap
            max_gap = min_dist;
        end
    end
end

function [inliers endline m b] = calc_inliers(cart, d, endpoints)
% calculates the inliers on the line segment as well as where the segment
% ends given the two random endpoints to make a line through, datapoints
% cart, and inlier distance

    m = (endpoints(1,2)-endpoints(2,2))/(endpoints(1,1)-endpoints(2,1)); % rise/run
    b = endpoints(1,2)-m*endpoints(1,1); % b = y - mx
    endline = endpoints; % intersection on line tangent to endpoints
    
    inliers = [];
    for i=1:size(cart, 1)
        % calculate (x,y) point along line closest to cart(i) & dist between
        bn = cart(i, 2) + cart(i, 1)/m; % solve b = y + x/m
        x = (bn - b)/(m + 1/m); % solve mx+b = -x/m+bn for x
        y = m*x + b;
        dist_tan = sqrt((cart(i,1)-x)^2 + (cart(i,2)-y)^2); % sqrt(dx^2 + dy^2)
        
        if dist_tan <= d % if it's an inlier
            inliers = [inliers; i]; % add new inlier to list
            
            % if it's the first inlier, make it the first endpoint
            if size(inliers, 1) == 1
                endpoints(1,:) = cart(i,:);
                endline(1,:) = [x y];
            end
            % if it's the second inlier, assign the leftmost as endpoint 1
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
            % if it's another inlier, figure out if it's farthest left or right so far
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
    % circle fit using discriminating criteria (point distance, radius,
    % near inliers) to identify the best circle in datapoints 'cart'
    
    calc_r = r;
    inliers = zeros(0,2);
    center = zeros(0,2);
    for k=1:n
%         % Looking in a particular area on the graph instead of guessing
%         % blindly. signifiantly improves recognition at the expense of
%         % having to know map layout.
%         xs = cart(:,1);
%         ys = cart(:,2);
%         indexx=find(xs > .2 & xs < .8);
%         indexy=find(ys > -2.8 & ys < -2.2);
%         index_circ=intersect(indexx,indexy);
%         circx=xs(index_circ);
%         circy=ys(index_circ);
%         A = [circx circy ones(size(circx))];
%         b = -circx.^2 - circy.^2;
        
        d1 = 2*r+1; d2 = 2*r+1;
        while d1 > 2*r || d2 > 2*r % check whether distance apart is too big
            % Pick 3 random points
            candidates = datasample(cart, 3, 'Replace', false);
            d1 = sqrt((candidates(1,1)-candidates(2,1)).^2 + (candidates(1,2)-candidates(2,2)).^2);
            d2 = sqrt((candidates(2,1)-candidates(3,1)).^2 + (candidates(2,2)-candidates(3,2)).^2);
        end
        
        % linear regression, solving for center and radius through candidates
        A = [candidates(:,1) candidates(:,2) ones(size(candidates, 1),1)];
        b = -candidates(:,1).^2 - candidates(:,2).^2;
        w = A\b;
        new_center = [-w(1)/2 -w(2)/2];
        new_r = sqrt(new_center(1).^2 + new_center(2).^2 - w(3));
        
        if abs(new_r - r) < tol % continue if radius is reasonably correct
            % counting up the inliers and near inliers
            near_inliers = [];
            new_inliers = [];
            for i=1:size(cart, 1)
                dist = abs(sqrt((cart(i,1)-new_center(1)).^2 + (cart(i,2)-new_center(2)).^2) - new_r);
                if dist < d
                    new_inliers = [new_inliers; i];
                end
                
                % near inliers is important bc circles don't have them, but
                % circle fits over linear segments do.
                if dist < 2*d && dist > d
                    near_inliers = [near_inliers; i];
                end
            end
            
            % see if it's a better match than the previous one
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