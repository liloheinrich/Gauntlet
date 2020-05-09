% plot the path from encoder data

hold on
load('output')
load('pathpoints')
load('shapedata')
dt = .1;
d=.235;

count = 1;
dx = xn(count+1)-xn(count);
dy = yn(count+1)-yn(count);
theta = atan2(dy, dx);
    
V = diff(dataset(:,3)+dataset(:,2))/2 /dt;
w = diff(dataset(:,3)-dataset(:,2))/d /dt;
r = zeros(size(dataset,1), 2);

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