clc; clf; close all; clear all;
load shapedata;

%% numerically solve for potential field and gradient field over a meshgrid
[X Y] = meshgrid(-3:.2:3);
Z4 = 0;
inc = .1;
for i=1:size(endpoints, 1)
    for j = min(endpoints(i,:,1)):inc:max(endpoints(i,:,1))
        for k = min(endpoints(i,:,2)):inc:max(endpoints(i,:,2))
            Z4 = Z4 + log(sqrt((X-j).^2+(Y-k).^2));
        end
    end
end
for theta = 0:inc:2*pi
    a = center(1)+radius.*cos(theta);
    b = center(2)+radius.*sin(theta);
    Z4 = Z4 - log(sqrt((X-a).^2+(Y-b).^2))*2;
end
[U V] = gradient(Z4, .2, .2);
U(isinf(U)) = 0;
V(isinf(V)) = 0;
contour(X, Y, Z4, 'levels', 20)
hold on
quiver(X, Y, U, V)
hold off
graph(radius, center, endpoints)
title('Identified shapes from initial position scan with potential field contours and gradient field vectors')

%% generate path points using this parameterized function
lambda = .001; sigma = .9575;
xi = 0; yi = 0; max_pts = 100;
[xn, yn] = calculate_all('shapedata', xi, yi, lambda, sigma, max_pts)
save('pathpoints', 'xn', 'yn')

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
        Z = Z - log(sqrt((X-a).^2+(Y-b).^2))*3;
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