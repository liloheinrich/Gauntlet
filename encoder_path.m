% used to plot the path from encoder data and compare to calculated path

hold on
load('output')
load('pathpoints')
load('shapedata')

dt = .1; % time increment
d=.235; % wheelbase width
dx = xn(2)-xn(1);
dy = yn(2)-yn(1);
theta = atan2(dy, dx);
    
V = diff(dataset(:,3)+dataset(:,2))/2 /dt; % linear velocity
w = diff(dataset(:,3)-dataset(:,2))/d /dt; % angular velocity
r = zeros(size(dataset,1), 2);

% reconstruct path r from V and w with start angle theta
for i=2:size(dataset,1) 
    theta = theta + w(i-1)*dt;
    r(i,1) = r(i-1,1) + cos(theta)*V(i-1)*dt; 
    r(i,2) = r(i-1,2) + sin(theta)*V(i-1)*dt;
end

plot(r(:,1), r(:,2), 'b--')
plot(xn, yn, 'g')
ylabel('y distance (m)')
xlabel('x distance (m)')
title('Calculated and measured path')
xlim([-2 3])
ylim([-3 1])
axis equal
hold off
graph(radius, center, endpoints)
legend('Measured Path', 'Calculated Path')

% graph gauntlet layout for context on the graph
function graph(radius, center, endpoints)
    hold on
    
    % calculate equidistant points along circle perimeter
    circlepts = zeros(2,360);
    for angle=1:360
        circlepts(:,angle) = [radius*cosd(angle)+center(1), radius*sind(angle)+center(2)];
    end
    plot(circlepts(1,:), circlepts(2,:), 'm') % circle
    plot(center(:,1), center(:,2), 'mx') % circle center
    plot(0,0,'bx') % start point
    for i=1:size(endpoints, 1)
        plot(endpoints(i,:,1), endpoints(i,:,2), 'r') % all the line segments
    end
    axis equal;
    xlim([-2, 3]);
    ylim([-3, 1]);
    xlabel('[m]')
    ylabel('[m]')
    hold off;
end