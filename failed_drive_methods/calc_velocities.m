%% Load all vars

% Define all the path params
syms b t
b_num = 0.1;
bt_max = 0.9;
len = 100;

load pathpoints
p = polyfit(xn, yn, 7);
y1 = polyval(p,xn);
figure
plot(xn,yn,'o')
hold on; plot(xn,y1, 'g'); hold off;
n = b*t;
R = [t; p(1)*n^7+p(2)*n^6+p(3)*n^5+p(4)*n^4+p(5)*n^3+p(6)*n^2+p(7)*n+p(8); 0]; % path function in terms of t

d = .235; % wheelbase width in m

% Load up measured data
% load('BODdata');
% dataset = dataset(35:size(dataset,1)-15, :);
% dataset(:,1) = dataset(:,1) - dataset(1,1);
% dt = mean(diff(dataset(:,1)));

%% symbolically calculate functions
dR=diff(R,t); % tangent vector (lin vel)
T_hat=simplify(dR./norm(dR)); % unit tangent vector
dT_hat=diff(T_hat,t); % deriv of tangent vector
N_hat=simplify(dT_hat); % normal vector
B_hat=simplify(cross(T_hat,N_hat)); % binormal vector
%kappa=simplify(norm(dT_hat)/norm(dR)); % curvature
%tau=-N_hat*(diff(B_hat,t)/norm(dR))'; % torsion

lin_vec = dR; % linear velocity in tangent direction
ang_vec = B_hat*simplify(norm(cross(T_hat,dT_hat))); % angular velocity in k direction
lin = norm(lin_vec); % linear velocity
ang = norm(ang_vec); % angular velocity
right = lin + ang*d/2; % right wheelvelocity
left = lin - ang*d/2; % left wheelvelocity
lin, ang, right, left

%% plot 3D path over time, showing angular & linear velocity vectors
t_num = linspace(0,bt_max/b_num,len);
ang_num = zeros(len, 1);
lin_num = zeros(len, 1);
right_num = zeros(len, 1);
left_num = zeros(len, 1);

for n=1:len
    R_num(n,:)=double(subs(R,[b, t],[b_num, t_num(n)]));
    T_dir=double(subs(T_hat,[b, t],[b_num, t_num(n)]));
    B_dir=double(subs(B_hat,[b, t],[b_num, t_num(n)]));
    
    lin_num(n)=double(subs(lin,[b, t],[b_num, t_num(n)]));
    ang_num(n)=double(subs(ang,[b, t],[b_num, t_num(n)]))*B_dir(3);
    right_num(n)=double(subs(right,[b, t],[b_num, t_num(n)]));
    left_num(n)=double(subs(left,[b, t],[b_num, t_num(n)]));
    
    plot3(R_num(:,1),R_num(:,2),R_num(:,3)), axis([-4 4 -4 4 -4 4]), hold on
    quiver3(R_num(n,1),R_num(n,2),R_num(n,3),lin_num(n)*T_dir(1),lin_num(n)*T_dir(2),0,'r')
    quiver3(R_num(n,1),R_num(n,2),R_num(n,3),0,0,ang_num(n),'g'), hold off
    drawnow
end

%% plot the path
% [timestamp, positionLeft, positionRight, AccelX, AccelY, AccelZ];
hold on
plot(R_num(:,1), R_num(:,2), 'b')

load('BODdata')
V = diff(dataset(:,3)+dataset(:,2))/2 /dt;
w = diff(dataset(:,3)-dataset(:,2))/d /dt;
r = zeros(size(dataset,1), 2);

bridgeStart = double(subs(R,t,0));
startingThat = double(subs(That,t,0));
r(1,:) = bridgeStart(1:2);
theta = atan2(startingThat(2), startingThat(1));
for i=2:size(dataset,1)
    theta = theta + w(i-1)*dt;
    r(i,1) = r(i-1,1) + cos(theta)*V(i-1)*dt;
    r(i,2) = r(i-1,2) + sin(theta)*V(i-1)*dt;
end
plot(r(:,1), r(:,2), 'b--')
ylabel('y distance (m)')
xlabel('x distance (m)')
legend('Calculated path', 'Measured Path')
title('Parametric path')
axis equal
hold off

%% plot angular & linear velocity
clf('reset')
hold on
w = diff(dataset(:,3)-dataset(:,2))/d /dt;
V = diff(dataset(:,3)+dataset(:,2))/2 /dt;
plot(t_num, lin_num, 'r')
plot(dataset(1:size(dataset,1)-1), V, 'r--')
plot(t_num, ang_num, 'b')
plot(dataset(1:size(dataset,1)-1), w, 'b--')
axis equal
xlabel('Time (s)')
ylabel('Velocity (m/s or deg/s)')
legend('Calculated linear', 'Measured linear', 'Calculated angular', 'Measured angular')
title('Linear & angular velocity')
hold off

%% plot the left & right velocities
clf('reset')
hold on
plot(t_num, right_num, 'r')
plot(dataset(1:size(dataset,1)-1), diff(dataset(:,3))/dt, 'r--')
plot(t_num, left_num, 'b')
plot(dataset(1:size(dataset,1)-1), diff(dataset(:,2))/dt, 'b--')
xlabel('Time (s)')
ylabel('Velocity (m/s)')
legend('Right calculated', 'Right Measured', 'Left calculated', 'Left Measured')
title('Wheel velocity')
hold off

%% plot the right & left distance
% [timestamp, positionLeft, positionRight, AccelX, AccelY, AccelZ];
hold on
load('BODdata')
plot(dataset(:,1), dataset(:,2)-dataset(1,2), 'b')
plot(dataset(:,1), dataset(:,3)-dataset(1,3), 'r')
ylim([-1 15])
ylabel('Distance (m)')
xlabel('Time (s)')
legend('Left wheel', 'Right wheel')
title('Measured distance from encoder data')
hold off

