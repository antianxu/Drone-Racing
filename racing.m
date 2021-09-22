clear all
close all
clc

%% Provided Code
% Aerial racing
I = imread('Racecourse.png');
map = im2bw(I, 0.4);  % Convert to 0-1 image
map = flipud(1-map)'; % Convert to 0 free, 1 occupied and flip.
[M,N]= size(map);     % Map size
% Robot start position
dxy = 0.1; % Resolution
startpos = dxy*[350,250]; % Starting Point
checkpoints = dxy*[440,620;440,665]; % Check Point

%% Constants
dt = 1/5;        % Update rate
std_p = 0.05;    % Standard deviation of x and y
std_th = 0.02;   % Standard deviation of theta
std_r  = 0.05;   % Standard deviation of measurement readings
v_max = 20;      % Maximum allowed velocity 20m/s
r_max = 10;      % Maximum measurement range
r_min = 0.3;     % Minimum measurement range
phi_max =  69/2*(pi/180); % Maximum angle in rad
phi_min = -69/2*(pi/180); % Minimum angle in rad
d_phi = (phi_max-phi_min)/127; % Angle resolution
mid_pt = mean(checkpoints, 1); % Check point middle point
rad_di = abs((checkpoints(2,2)-checkpoints(1,2))/2);  
% Motion Model used for updating
H = @(t) [cos(t),-sin(t),0;sin(t),cos(t),0;0,0,1];

%% Initialize
ROB = [startpos, 0]'; % Robot position [x, y, theta] 
ogp = zeros(M,N);     % Occupancy grid probability
oglo = zeros(M,N);    % Occupancy grid log probability
ang_row = phi_min:d_phi:phi_max;   % Angle vector 
lm = [mid_pt, rad_di; startpos, 10; startpos, 0.5]; % Landmarks
l = 1;              % Landmark trying to reach
loop_1 = ROB(1:2)'; % Storing path
state = 1; % 0 travelling and 1 rotating
o_ang = 0; % Starting angle when rotating
it = 0;    % Number of rotation until made full 360
t = 0;     % Time 
%% Mapping the Environment

% Start the video recording of the travel
vid = VideoWriter('ExplorationNoPath.mp4', 'MPEG-4');
vid2 = VideoWriter('ExplorationPath.mp4', 'MPEG-4');
open(vid);
open(vid2);

while l < size(lm, 1)+1
    % Make laser reading
    laser_scan = get_laser_scan(map, ROB, ang_row, r_max, dxy);
    laser_noise = randn(size(laser_scan))*std_r;
    
    % Add appropriate noise to the map
    laser_scan = laser_scan + laser_noise;
    
    % Update the map
    [ogp, oglo] = og_map_update(ogp, oglo, laser_scan, ROB, M, N); 
       
    % Controller
    % If the robot too close to the wall perform need to adjust the course
    val = mean(laser_scan);
    if (val < 2 || min(laser_scan) < 0.3) && state == 0
        % Tell the robot to stop and rotate on the spot
        state = 1;
        o_ang = ROB(3);
        it = 0;
        rot_vec = [0,0];
    end

    % Traveling stage
    if state == 0
        d_th1 = 0.005 * (sum(laser_scan(105:end) > 4) - sum(laser_scan(1:24) > 4));
        d_th2 = 0.00835 * (laser_scan(end)-max(laser_scan(64)*0.2/sin(ang_row(end)),0.5));
        d_th3 = 0.00835 * (laser_scan(1) - max(laser_scan(64)*0.2/sin(ang_row(end)),0.5));
        d_th  = d_th1 + d_th2+ d_th3;
        
        % After completing the lap return to the original position
        if l == size(lm, 1)
            d_th = atan2( lm(l,2) - ROB(2), lm(l,1) - ROB(1));
            d_th = (d_th- ROB(3)); 
            d_th = mod(d_th,2*pi);
        end
        u = [[mean(laser_scan(54:75))./7.5;0]*dt;  d_th];
        % Store point visited
        loop_1(end+1, :) = ROB(1:2)';
    
    % Rotating stage
    elseif state == 1
        it = it + 1;
        if it < 36
            u = [0; 0; -10 * pi/180];
            scor = mean(laser_scan) + 2*HistoryPenalty(loop_1(max(end-50, 1):end), [ROB(1) + 1*cos(ROB(3)), ROB(2)+ 1*sin(ROB(3))]);
            rot_vec(it,:) = [ROB(3), scor];
        else
            [scor, ind] = max(rot_vec(:, 2));
            if mod(rot_vec(ind,1) - ROB(3), 2*pi) > pi  
                u = [0; 0; -10 * pi/180];
            else
                u = [0; 0;  10 * pi/180];
            end
            if abs(rot_vec(ind,1) - ROB(3)) < 0.075
                state = 0;
            end
        end
    end
    
    % Update the position 
    ROB = ROB + H(ROB(3)) * u;
    ROB(3) = mod(ROB(3),2*pi);
    
    % Add noise to the position
    ROB = ROB + [std_p*randn(2,1); std_th*randn(1)];

    % Plot the Map
    Arena2Plot(ogp, ROB, loop_1, M, N, dxy, t)
    frame = getframe(figure(1));
    writeVideo(vid, frame)
    frame = getframe(figure(2));
    writeVideo(vid2, frame)
    
    % Check if the robot near the landmark
    if norm(ROB(1:2) - lm(l,1:2)') < lm(l,3)
        l = l + 1;
    end 
    t = t + dt;
end

% Rotating back to the original position
while abs(ROB(3)) > 0.075
    d_th = min(abs(ROB(3)), 5*pi/180);
    if ROB(3) < pi  
        u = [0;0;-d_th];
    else
        u = [0;0; d_th];
    end
    ROB = ROB + H(ROB(3)) * u;
    ROB(3) = mod(ROB(3),2*pi);
    
    % Add noise to the position
    ROB = ROB + [std_p*randn(2,1); std_th*randn(1)];
    
    % Record the position
    Arena2Plot(ogp, ROB, loop_1, M, N, dxy, t)
    frame = getframe(figure(1));
    writeVideo(vid, frame)
    frame = getframe(figure(2));
    writeVideo(vid2, frame)
    
    t = t + dt;
end

% Stop Recording
close(vid);
close(vid2);

%% For plotting the six evenly spaced points in time around theloop
close all
vid = VideoReader('ExplorationPath.mp4');
f_pts = 1*t/(6*dt) * ones(1,5);
f_pts = floor(cumsum(f_pts)); % Floor since can not have floating frame

% Reading frames in
frame1 = read(vid,f_pts(1));
frame2 = read(vid,f_pts(2));
frame3 = read(vid,f_pts(3));
frame4 = read(vid,f_pts(4));
frame5 = read(vid,f_pts(5));
frame6 = read(vid,Inf);

% Ploting
figure(1); imshow(frame1)
figure(2); imshow(frame2)
figure(3); imshow(frame3)
figure(4); imshow(frame4)
figure(5); imshow(frame5)
figure(6); imshow(frame6)

%% Store Information as not to rerun entire thing
save('Loop1Data.mat', 't', 'f_pts', 'ogp', 'oglo', 'loop_1')


%% Load 
clc; clear all; close all
load 'Loop1Data.mat'



%% Provided Code
% Aerial racing
I = imread('Racecourse.png');
map = im2bw(I, 0.4);  % Convert to 0-1 image
map = flipud(1-map)'; % Convert to 0 free, 1 occupied and flip.
[M,N]= size(map);     % Map size
% Robot start position
dxy = 0.1; % Resolution
startpos = dxy*[350,250]; % Starting Point
checkpoints = dxy*[440,620;440,665]; % Check Point

%% Constants
dt = 1/5;        % Update rate
std_p = 0.05;    % Standard deviation of x and y
std_th = 0.02;   % Standard deviation of theta
std_r  = 0.05;   % Standard deviation of measurement readings
v_max = 20;      % Maximum allowed velocity 20m/s
r_max = 10;      % Maximum measurement range
r_min = 0.3;     % Minimum measurement range
phi_max =  69/2*(pi/180); % Maximum angle in rad
phi_min = -69/2*(pi/180); % Minimum angle in rad
d_phi = (phi_max-phi_min)/127; % Angle resolution
mid_pt = mean(checkpoints, 1); % Check point middle point
rad_di = abs((checkpoints(2,2)-checkpoints(1,2))/2);  
% Motion Model used for updating
H = @(t) [cos(t),-sin(t),0;sin(t),cos(t),0;0,0,1];



%% Fastest Lap
% %Initialize
% ROB = [startpos,0]';% Drone original Pose
% b_map = ogp;        % Use the built map for A* algorithm search 
% % Use 6 previous points along the way that the 
% lm = [loop_1(f_pts(1),:); ...
%       loop_1(f_pts(2),:); ...
%       loop_1(f_pts(3),:); ...
%       loop_1(f_pts(4),:); ...
%       loop_1(f_pts(5),:); ...
%       startpos]./dxy;
% lm = round(lm); % A* needs int not float value
% dis = [10,10,10,10,10, 1]; % Tolerance how far 
% 
% % Use A* to generate the path
% loop_2 = A_star(ROB(1:2)./dxy, lm(1,:), b_map, 10);
% for l= 1:size(lm)-1
%    seg_path = A_star(lm(l,:), lm(l+1,:), b_map, dis(l)); 
%    loop_2(end+1:end+size(seg_path, 1), :) = seg_path; 
% end
% loop_2 = loop_2.*dxy; % Convert points to the 
% 
% % Moving Average Filter
% windowSize = 5; 
% b = (1/windowSize)*ones(1,windowSize);
% a = 1;
% loop_2 = [filter(b,a,loop_2(:,1)), filter(b,a,loop_2(:,2))];
% loop_2 = loop_2(windowSize:end, :);
% vid = VideoWriter('FastLap.mp4', 'MPEG-4');
% open(vid);
% t = 0;
% for it = 1:size(loop_2,1)
%     % Plot the map    
%     d_th = atan2( loop_2(it,2) - ROB(2), loop_2(it,1) - ROB(1));
%     d_th = mod(d_th,2*pi);
%     ROB = [loop_2(it, :)'; d_th] ;
%     ArenaPlot(b_map, ROB,  M, N, dxy, t)
%     frame = getframe(gcf);
%     writeVideo(vid, frame)
%     t = t + dt;
% end
% 
% % Rotating back to the original position
% ROB(3) = 0;
% ArenaPlot(b_map,ROB, [0,0],  M, N, dxy)
% frame = getframe(gcf);
% writeVideo(vid, frame)
% 
% 
% % Stop Recording
% close(vid);

%% Path Planning
%Initialize
ROB = [startpos,0]';% Drone original Pose
b_map = ogp;        % Use the built map for A* algorithm search 
% Use 6 previous points along the way that the 
lm = [loop_1(f_pts(1),:); ...
      loop_1(f_pts(2),:); ...
      loop_1(f_pts(3),:); ...
      loop_1(f_pts(4),:); ...
      loop_1(f_pts(5),:); ...
      startpos]./dxy;
lm = round(lm); % A* needs int not float value
dis = [10,10,10,10,10, 1]; % Tolerance how far 


%% Use A* to generate the path
loop_2 = A_star(ROB(1:2)./dxy, lm(1,:), b_map, 10);
for l= 1:size(lm)-1
   seg_path = A_star(lm(l,:), lm(l+1,:), b_map, dis(l)); 
   loop_2(end+1:end+size(seg_path, 1), :) = seg_path; 
end
loop_2 = loop_2.*dxy; % Convert points to the 

% Moving Average Filter for the path
windowSize = 5; 
b = (1/windowSize)*ones(1,windowSize);
a = 1;
loop_2 = [filter(b,a,loop_2(:,1)), filter(b,a,loop_2(:,2))];
loop_2 = loop_2(windowSize:end, :);

% Store the data
save('Loop2Path.mat', 'loop_2')


%% Fast lap
load 'Loop1Data.mat'
load 'Loop2Path.mat'

ROB = [startpos,0]'; % Drone original Pose
b_map = ogp;        % Use the built map for plotting 
vid = VideoWriter('FastLap.mp4', 'MPEG-4');
open(vid);
t = 0;
it2 = 1;
dis = 1*ones(size(loop_2, 1), 1);
dis(end) = 0.05;

while it2 < size(loop_2,1)
    while norm(ROB(1:2) - loop_2(it2,1:2)') < dis(it2)
        it2 = it2 + 1;
        if it2 == size(loop_2,1)-1
            break
        end
    end
    
    % Controller
    des_th = atan2( loop_2(it2,2) - ROB(2), loop_2(it2,1) - ROB(1));
    d_th = (des_th- ROB(3)); 
    d_th = mod(d_th,2*pi);
      
    % Speed
    v = 1*(loop_2(it2,1:2)' - ROB(1:2));
    if norm(v) > 20
        v = 20 * v./norm(v);
    end
    
    % Restrict the angular velocity to 1*pi/180 m/s
    if d_th < pi
        d_th = min(d_th, 5*pi/180);
    else
        d_th = max(d_th, 355*pi/180);
    end
    
    % Velocity
    u = [v; d_th]; 
    
    % Update
    ROB = ROB + H(d_th) * u;
    ROB(3) = mod(ROB(3),2*pi);
    
    % Add noise to the position
    ROB = ROB + [std_p*randn(2,1); std_th*randn(1)];
    
    % Mapping
    ArenaPlot(b_map, ROB,  M, N, dxy, t)
    frame = getframe(gcf);
    writeVideo(vid, frame)
    t = t + dt;
end


% Rotating back to the original position
while abs(ROB(3)) > 0.075
    d_th = min(abs(ROB(3)), 10*pi/180);
    if ROB(3) < pi  
        u = [0;0;-d_th];
    else
        u = [0;0; d_th];
    end
    ROB = ROB + H(ROB(3)) * u;
    ROB(3) = mod(ROB(3),2*pi);
    
    % Add noise to the position
    ROB = ROB + [std_p*randn(2,1); std_th*randn(1)];
    
    % Record the position
    ArenaPlot(b_map, ROB,  M, N, dxy, t)
    frame = getframe(figure(1));
    writeVideo(vid, frame)
    
    t = t + dt;
end


% Rotating back to the original position
%ROB(3) = 0;
%ArenaPlot(b_map,ROB, [0,0],  M, N, dxy)
%frame = getframe(gcf);
%writeVideo(vid, frame)


% Stop Recording
close(vid);

%% Comparing two laps
loop_2 = [startpos;loop_2];
loop_2 = [loop_2;startpos];
PathPlot(ogp, [startpos,0],loop_1, loop_2, M, N, dxy)

%% Helper Functions
% Penalty calculation
function [pen] = HistoryPenalty(prev,rob)
    diff = sqrt((prev(:,1) - rob(1)).^2 + (prev(:,2) - rob(2)).^2);
    pen = mean(mean(diff,2));
end

% Ploting of path and no path racing
function Arena2Plot(map, rob, path, M, N, res, t)
    figure(1); clf; hold on;
    colormap('gray');
    imagesc(1-map');
    plot( rob(1)/res,  rob(2)/res, 'ro',  'MarkerSize',5, 'LineWidth', 3);
    quiver(rob(1)/res, rob(2)/res, 25*cos(rob(3)), 25*sin(rob(3)), 'k', 'MaxHeadSize', 2)
    title(['Time ', num2str(floor((t+0.1)/60)),' minutes ',num2str(mod(t,59.9)),' seconds'])
    xlabel('North (decimeters)')
    ylabel('East  (decimeters)')
    axis equal
    xlim([1 M])
    ylim([1 N])
    
    figure(2); clf; hold on;
    colormap('gray');
    imagesc(1-map');
    plot(   rob(1)/res,     rob(2)/res, 'ro', 'LineWidth', 3, 'MarkerSize',5);
    plot(path(:,1)/res,  path(:,2)/res, 'g-', 'LineWidth', 3);
    quiver(rob(1)/res, rob(2)/res, 25*cos(rob(3)), 25*sin(rob(3)), 'k', 'MaxHeadSize', 2)
    title(['Time ', num2str(floor((t+0.1)/60)),' minutes ',num2str(mod(t,59.9)),' seconds'])
    xlabel('North (decimeters)')
    ylabel('East  (decimeters)')
    axis equal
    xlim([1 M])
    ylim([1 N])
end

% Ploting of no path racing
function ArenaPlot(map, rob, M, N, res, t)
    figure(1); clf; hold on;
    colormap('gray');
    imagesc(1-map');
    plot( rob(1)/res,  rob(2)/res, 'ro',  'MarkerSize',5, 'LineWidth', 3);
    quiver(rob(1)/res, rob(2)/res, 25*cos(rob(3)), 25*sin(rob(3)), 'k', 'MaxHeadSize', 2)
    title(['Time ', num2str(floor((t+0.1)/60)),' minutes ',num2str(mod(t,59.9)),' seconds'])
    xlabel('North (decimeters)')
    ylabel('East  (decimeters)')
    axis equal
    xlim([1 M])
    ylim([1 N])
end

% Plotting for comparing
function PathPlot(map, rob, path1, path2, M, N, res)
    figure(1); clf; hold on;
    colormap('gray');
    imagesc(1-map');
    p1 = plot(path1(:, 1)/res, path1(:, 2)/res, 'g-', 'LineWidth', 2);
    p2 = plot(path2(:, 1)/res, path2(:, 2)/res, 'b-', 'LineWidth', 2);
    plot( rob(1)/res,  rob(2)/res, 'ro',  'MarkerSize',5, 'LineWidth', 3);
    quiver(rob(1)/res, rob(2)/res, 25*cos(rob(3)), 25*sin(rob(3)), 'k', 'MaxHeadSize', 2)
    title('Drone Path')
    xlabel('North (decimeters)')
    ylabel('East  (decimeters)')
    legend([p1; p2],['Mapping Path'; 'Fastest Path'])
    axis equal
    xlim([1 M])
    ylim([1 N])
end